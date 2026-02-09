//


#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>
#include <CORA/CORA_types.h>

#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>
#include <Eigen/Dense>

using CORA::InstrumentationFunction;
using CORA::CoraTntResult;
using  CORA::CertResults;

void printIfVerbose(bool verbose, std::string msg) {
  if (verbose) {
    std::cout << msg << std::endl;
  }
}

CORA::Scalar thresholdVal(CORA::Scalar val, CORA::Scalar lower_bound,
                          CORA::Scalar upper_bound) {
  if (val < lower_bound) {
    return lower_bound;
  } else if (val > upper_bound) {
    return upper_bound;
  } else {
    return val;
  }
}


//
using Matrix = Eigen::MatrixXd; // or your typedef
using Scalar = Matrix::Scalar;

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include "json.hpp"   // nlohmann::json
using json = nlohmann::json;

using namespace std;
struct Config {
  int init_rank_jump;
  int max_rank;
  bool verbose;
  bool log_iterates;
  bool show_iterates;
  double step_size;
  int max_iter;
  double theta;
  std::vector<std::string> files;
};


// -----------------------------
// MAIN
// -----------------------------
int main(int argc, char **argv) {

  // reading and getting the paras from the config.json
  json cfg;
  Config config ;
  std::ifstream file("/home/raoui/cora-main/examples/my_labs/config.json");
  file >> cfg;

  config.init_rank_jump = cfg["init_rank_jump"];
  config.max_rank = cfg["max_rank"];
  config.verbose = cfg["verbose"];
  config.log_iterates = cfg["log_iterates"];
  config.show_iterates = cfg["show_iterates"];
  config.step_size = cfg["step_size"];
  config.max_iter     = cfg["max_iter"];
  config.theta     = cfg["theta_init"];
  std::string traj_file = cfg["trajectory_file"];

  std::string file_path = "/home/raoui/Music/cora-main/examples/data/single_drone.pyfg";

  CORA::Problem problem = CORA::parsePyfgTextToProblem(file_path);
  problem.updateProblemData();

  CORA::Matrix x0 = problem.getRandomInitialGuess();

  int max_rank = 10;



  // objective function   Y  is the low-rank factor of the SDP matrix
  Optimization::Objective<Matrix, Scalar, Matrix> f =
      [&problem](const Matrix &Y, const Matrix &NablaF_Y) {
        return problem.evaluateObjective(Y);
  };

// I have to continue on coding the solver
// the solver needs Riemanian gradient and riemanian hessian vector products
  Optimization::Riemannian::QuadraticModel<Matrix, Matrix, Matrix> QM =
        [&problem](const Matrix &Y, Matrix &grad,
                   Optimization::Riemannian::LinearOperator<Matrix, Matrix,
                                                            Matrix> &HessOp,
                   Matrix &NablaF_Y) {
          // Compute and cache Euclidean gradient at the current iterate
          NablaF_Y = problem.Euclidean_gradient(Y);

          // Compute Riemannian gradient from Euclidean gradient
          grad = problem.Riemannian_gradient(Y, NablaF_Y);

          // Define linear operator for computing Riemannian Hessian-vector
          // products (cf. eq. (44) in the SE-Sync tech report)
          HessOp = [&problem](const Matrix &Y, const Matrix &Ydot,
                              const Matrix &NablaF_Y) {
            return problem.Riemannian_Hessian_vector_product(Y, NablaF_Y, Ydot);
          };
  };

  // get retraction from problem
  Optimization::Riemannian::Retraction<Matrix, Matrix, Matrix> retract =
      [&problem](const Matrix &Y, const Matrix &V, const Matrix &NablaF_Y) {
        return problem.retract(Y, V);
  };

  // Euclidean gradient (is passed by reference to QM for caching purposes)
  Matrix NablaF_Y;

  // get preconditioner from problem
  std::optional<
      Optimization::Riemannian::LinearOperator<Matrix, Matrix, Matrix>>
      precon = [&problem](const Matrix &Y, const Matrix &Ydot,
                          const Matrix &NablaF_Y) {
        return problem.tangent_space_projection(Y, problem.precondition(Ydot));
  };

  bool show_iterates = false;


    // default TNT parameters for CORA
    Optimization::Riemannian::TNTParams<Scalar> params;
    params.Delta0 = 5;
    params.alpha2 = 3.0;
    params.max_TPCG_iterations = 80;
    params.max_iterations = 250;
    params.preconditioned_gradient_tolerance = 1e-6;
    params.gradient_tolerance = 1e-6;
    params.theta = 0.8;
    params.Delta_tolerance = 1e-5;
    params.verbose = show_iterates;
    params.precision = 2;
    params.max_computation_time = 20;
    params.relative_decrease_tolerance = 1e-6;
    params.stepsize_tolerance = 1e-6;
    params.log_iterates = config.log_iterates;

  // certification parameters
  const Scalar MIN_CERT_ETA = 1e-7;
  const Scalar MAX_CERT_ETA = 1e-1;
  const Scalar REL_CERT_ETA = 5e-6;
  const int LOBPCG_BLOCK_SIZE = 10;
  Scalar eta;



const  int max_relaxation_rank = 10;
  // metric over the tangent space is the standard matrix trace inner product
  Optimization::Riemannian::RiemannianMetric<Matrix, Matrix, Scalar, Matrix>
      metric =
          [](const Matrix &Y, const Matrix &V1, const Matrix &V2,
             const Matrix &NablaF_Y) { return (V1.transpose() * V2).trace(); };

  // no custom instrumentation function for now
  std::optional<InstrumentationFunction> user_function = std::nullopt;

  CoraTntResult result;

  //initialization

  Matrix X = problem.projectToManifold(x0);





  CertResults cert_results;
  Matrix eigvec_bootstrap;
  std::vector<Matrix> iterates = std::vector<Matrix>();
  bool first_loop = true;
  int loop_cnt = 0;
  while (problem.getRelaxationRank() <= max_relaxation_rank) {
    loop_cnt++;
    // solve the problem
    printIfVerbose(config.verbose, "\nSolving problem at rank " +
                                std::to_string(problem.getRelaxationRank()));

    if (config.log_iterates) {
      for (Matrix iterate : result.iterates) {
        // check that the iterate is the expected size
        checkMatrixShape(
            "solveCora::iterate", problem.getExpectedVariableSize(),
            problem.getRelaxationRank(), iterate.rows(), iterate.cols());
        iterates.push_back(iterate);
      }
    }


    // check if the solution is certified
    eta = thresholdVal(result.f * REL_CERT_ETA, MIN_CERT_ETA, MAX_CERT_ETA);
    if (first_loop) {
      eigvec_bootstrap = result.x;

      // if we are using the translation implicit formulation, we should solve
      // for the translation explicit solution (there is an analytical solution
      // for this)
      if (problem.getFormulation() == Formulation::Implicit) {
        eigvec_bootstrap =
            problem.getTranslationExplicitSolution(eigvec_bootstrap);
      }

    } else {
      eigvec_bootstrap = cert_results.all_eigvecs;
    }
  }
  return 0;
}
