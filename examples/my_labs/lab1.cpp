//


#include <CORA/CORA.h>
#include <CORA/CORA_utils.h>

#include <Optimization/Base/Concepts.h>
#include <Optimization/Riemannian/TNT.h>


//


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
// Small 2D vector
// -----------------------------
struct Vec2 {
  double x, y;
};

// -----------------------------
// SO(2) rotation
// -----------------------------
Vec2 rotate(const Vec2& p, double theta) {
  return {
    std::cos(theta) * p.x - std::sin(theta) * p.y,
    std::sin(theta) * p.x + std::cos(theta) * p.y
  };
}

// -----------------------------
// Cost function
// -----------------------------
double cost(double theta,
            const std::vector<Vec2>& P,
            const std::vector<Vec2>& Q) {
  double c = 0.0;
  for (size_t i = 0; i < P.size(); i++) {
    Vec2 Rp = rotate(P[i], theta);
    double dx = Rp.x - Q[i].x;
    double dy = Rp.y - Q[i].y;
    c += dx*dx + dy*dy;
  }
  return c;
}

// -----------------------------
// Riemannian gradient on SO(2)
// -----------------------------
double gradient(double theta,
                const std::vector<Vec2>& P,
                const std::vector<Vec2>& Q) {

  double g = 0.0;
  for (size_t i = 0; i < P.size(); i++) {
    Vec2 Rp = rotate(P[i], theta);

    // derivative of R(theta)p wrt theta
    Vec2 dRp {
      -std::sin(theta)*P[i].x - std::cos(theta)*P[i].y,
       std::cos(theta)*P[i].x - std::sin(theta)*P[i].y
    };

    g += 2.0 * ((Rp.x - Q[i].x) * dRp.x +
                (Rp.y - Q[i].y) * dRp.y);
  }
  return g;
}

// -----------------------------
// MAIN
// -----------------------------
int main() {

  // reading and getting the paras from the config.json
  json cfg;
  Config config ;
  std::ifstream file("config.json");
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

  // -------- Load trajectory --------
  std::ifstream traj(traj_file);

  std::vector<Vec2> P, Q;

  // add the trajectory of the robot


  while (!traj.eof()) {
    Vec2 p, q;
    traj >> p.x >> p.y >> q.x >> q.y;
    if (traj) {
      P.push_back(p);
      Q.push_back(q);
    }
  }

    double theta = config.theta;

  // -------- Riemannian optimization --------
  cout<<"config.max_iter"<<config.max_iter<<endl;
  for (int k = 0; k < config.max_iter; k++) {
    double g = gradient(theta, P, Q);

    // Riemannian update on SO(2)
    theta -= config.step_size * g;

    // optional wrap angle
    theta = std::atan2(std::sin(theta), std::cos(theta));

    if (k % 20 == 0) {
      std::cout << "iter " << k
                << "  cost = " << cost(theta, P, Q)
                << "  theta = " << theta << std::endl;
    }
  }

  std::cout << "\nEstimated rotation (rad): " << theta << std::endl;
  std::cout << "Estimated rotation (deg): "
            << theta * 180.0 / M_PI << std::endl;

  return 0;
}
