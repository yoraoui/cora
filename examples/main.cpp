#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/pyfg_text_parser.h>
#include <fstream>

#ifdef GPERFTOOLS
#include <gperftools/profiler.h>
#endif

int main(int argc, char **argv) {
  if (argc != 2) {
    std::cout << "Usage: " << argv[0] << " [input .g2o file]" << std::endl;
    exit(1);
  }
  std::cout<<"hello"<<"\n";

  CORA::Problem problem = CORA::parsePyfgTextToProblem(argv[1]);
  problem.updateProblemData();

#ifdef GPERFTOOLS
  ProfilerStart("cora.prof");
#endif

  CORA::Matrix x0 = problem.getRandomInitialGuess();
  int max_rank = 10;

  CORA::CoraResult soln = CORA::solveCORA(problem, x0, max_rank);
  CORA::Matrix aligned_soln = problem.alignEstimateToOrigin(soln.first.x);

  std::cout << "Solution: " << std::endl;
  std::cout << aligned_soln << std::endl;



// ------------------------------------
// Save solution to file
// ------------------------------------
std::ofstream out("aligned_solution.txt");

if (!out.is_open()) {
  std::cerr << "Error: cannot open file for writing\n";
  return 1;
}

out << aligned_soln << std::endl;
out.close();

std::cout << "Solution saved to aligned_solution.txt\n";

#ifdef GPERFTOOLS
  ProfilerStop();
#endif

}
