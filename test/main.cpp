#include "dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}


int main() {

    auto mi = DQ_MujocoInterface();
    mi.load_model("ur3e_scene.xml");
    std::vector<std::string> jointnames = mi.get_joint_names();
    int n = jointnames.size();

    while(!mi.should_close_window() && !kill_this_process)
    {
        double simstart = mi.get_simulation_time();
        while( mi.get_simulation_time() - simstart < 1.0/60.0 )
        {
            mi.trigger_next_simulation_step();
            double value = 0.2 * sin(8.0 * mi.get_simulation_time());
            VectorXd q_cmd = VectorXd::Constant(n, value);
            mi.set_joint_target_positions(jointnames, q_cmd);  //arm_sh0 for Spot arm

            std::cout<<"--------------------------------------"<<std::endl;
            std::cout<<"q:     "<<mi.get_joint_positions(jointnames).transpose()<<std::endl;
            std::cout<<"q_dot: "<<mi.get_joint_velocities(jointnames).transpose()<<std::endl;
            std::cout<<"tau  : "<<mi.get_joint_forces(jointnames).transpose()<<std::endl;
            std::cout<<"    "<<std::endl;
        }
        mi.render();
    }
    return 0;
}
