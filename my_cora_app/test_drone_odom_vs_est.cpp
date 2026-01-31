#include <iostream>

#include <CORA/CORA.h>
#include <CORA/CORA_problem.h>
#include <CORA/CORA_types.h>
#include <CORA/pyfg_text_parser.h>



struct RFIDMeasurement {
    int pose_id;
    int tag_id;
    double range;
};

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0]
                  << " rfid_trajectory.pyfg\n";
        return 1;
    }



    std::vector<Eigen::Vector2d> rfid_tags = {
    {20, 40}, {60, 40}, {100, 40}, {140, 40},
    {20, 80}, {60, 80}, {100, 80}, {140, 80},
    {20,120}, {60,120}, {100,120}, {140,120},
    {40, 60}, {80, 60}, {120, 60},
    {40,100}, {80,100}, {120,100}};

    std::vector<Eigen::Vector2d> trajectory = {
    {30, 70}, {35, 75}, {40, 80}, {45, 85}, {50, 90},
    {60, 95}, {70,100}, {80,105}, {90,100}, {100,95},
    {110,90}, {115,85}, {120,80},
    {118,70}, {115,60}, {110,50}, {100,45},
    {90, 50}, {80, 55}, {70, 60}, {60, 65},
    {55, 70}, {50, 75},
    {52, 85}, {55, 95}, {60,105},
    {70,110}, {80,115}, {90,110},
    {95,100}, {100,90}, {105,80},
    {108,70}, {110,60},
    {105,50}, {95,45}, {85,50},
    {75,55}, {65,60}, {55,65},
    {45,70}, {35,75}, {30,70}
};

std::vector<RFIDMeasurement> measurements;

double noise_std = 0.1;

for (size_t i = 0; i < trajectory.size(); ++i)
{
    for (size_t j = 0; j < rfid_tags.size(); ++j)
    {
        double d = (trajectory[i] - rfid_tags[j]).norm();

        if (d < 25.0)  // portée RFID
        {
            measurements.push_back({
                (int)i,
                (int)j,
                d
            });
        }
    }
}


    // --------------------------------------------------
    // 1) Parse .pyfg into a CORA problem
    // --------------------------------------------------
    CORA::Problem problem =
        CORA::parsePyfgTextToProblem(argv[1]);

    problem.updateProblemData();

    std::cout << "Problem loaded successfully.\n";
    
    // --------------------------------------------------
    // 2) Initial guess and solver parameters
    // --------------------------------------------------
    CORA::Matrix x0 = problem.getRandomInitialGuess();

    int max_rank = 10;   // Riemannian Staircase max lift

    // --------------------------------------------------
    // 3) Solve with CORA
    // --------------------------------------------------
    std::cout << "Solving with CORA...\n";

    CORA::CoraResult result =
        CORA::solveCORA(problem, x0, max_rank);

    // --------------------------------------------------
    // 4) Align solution (gauge freedom)
    // --------------------------------------------------
    CORA::Matrix X_aligned =
        problem.alignEstimateToOrigin(result.first.x);

    std::cout << "CORA finished successfully.\n";
    std::cout << "Solution matrix size: "
              << X_aligned.rows() << " x "
              << X_aligned.cols() << std::endl;

    return 0;
}
