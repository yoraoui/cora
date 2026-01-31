#include <iostream>
#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/pyfg_text_parser.h>



using json = nlohmann::json;

enum InitType { Random, Odom };

struct Config {
  int init_rank_jump;
  int max_rank;
  bool verbose;
  bool log_iterates;
  bool show_iterates;
  CORA::Preconditioner preconditioner;
  CORA::Formulation formulation;
  InitType init_type;
  std::vector<std::string> files;
};



std::vector<std::string> getRangeAndRpmMrclamFiles() {
  std::string base_dir = "data/mrclam/range_and_rpm/";
  std::vector<std::string> filenames = {
      "mrclam2.pyfg",
      "mrclam4.pyfg",
      // "mrclam5a.pyfg", "mrclam5b.pyfg", "mrclam5c.pyfg",
      "mrclam6.pyfg",
      "mrclam7.pyfg",
  };




int main(int argc, char **argv) {
  std::vector<std::string> original_exp_files = {
      "data/plaza1.pyfg", "data/plaza2.pyfg", "data/single_drone.pyfg",
      "data/tiers.pyfg"}; // TIERS faster w/ random init

  auto mrclam_range_and_rpm_files = getRangeAndRpmMrclamFiles();

  std::vector<std::string> files = {};

  // original experiments
  files.insert(files.end(), original_exp_files.begin(),
               original_exp_files.end());

  // mrclam range and rpm experiments
  files.insert(files.end(), mrclam_range_and_rpm_files.begin(),
               mrclam_range_and_rpm_files.end());

  // files = {"data/test.pyfg"};
  // files = {"data/tiers.pyfg"};

  // load file from environment variable "CORAFILE"
  if (const char *env_p = std::getenv("CORAFILE")) {
    std::cout << "Using file from environment variable: " << env_p << std::endl;
    files = {env_p};
  }

  Config config = parseConfig("/home/raoui/Videos/cora-main/examples/config.json");

  for (auto file : files) {
    CORA::Matrix soln = solveProblem(
        file, config.init_rank_jump, config.max_rank, config.preconditioner,
        config.formulation, config.init_type, config.verbose,
        config.log_iterates, config.show_iterates);
    std::cout << std::endl;
  }
}
