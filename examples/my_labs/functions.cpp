#include <vector>

struct Config {
  int init_rank_jump;
  int max_rank;
  bool verbose;
  bool log_iterates;
  bool show_iterates;
  std::vector<std::string> files;
};

