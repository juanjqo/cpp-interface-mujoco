#include "mujoco/mujoco.h"
#include <iostream>
#include <thread>
#include <iostream>
#include <memory>
#include "dqrobotics/interfaces/mujoco/DQ_MujocoInterface.h"

/*********************************************
 * SIGNAL HANDLER
 * *******************************************/
#include<signal.h>

static std::atomic_bool kill_this_process(false);

void sig_int_handler(int);

void sig_int_handler(int)
{
    kill_this_process = true;
}


int main() {

    auto mi = DQ_MujocoInterface();
    mi.load_model("scene.xml");
    while(!mi.should_close_window() && !kill_this_process)
    {
        double simstart = mi.get_simulation_time();
        while( mi.get_simulation_time() - simstart < 1.0/30.0 )
            mi.trigger_next_simulation_step();

        //std::cout<<"q: "<<mi.get_joint_positions({"arm_sh0"}).transpose()<<std::endl;
        VectorXd q_cmd = VectorXd::Zero(7);
        double value = 0.5 * sin(8.0 * mi.get_simulation_time());
        q_cmd << value, value,value, value,value, value, value;
        if (mi.get_simulation_time() > 10)
            mi.set_joint_target_positions({"fr3_joint1", "fr3_joint2","fr3_joint3", "fr3_joint4", "fr3_joint5", "fr3_joint6", "fr3_joint7"}, q_cmd);  //arm_sh0 for Spot arm


        mi.render();
    }
    return 0;
}
