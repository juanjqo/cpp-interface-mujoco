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
        while( mi.get_simulation_time() - simstart < 1.0/60.0 )
           mi.trigger_next_simulation_step();
        std::cout<<"q: "<<mi.get_joint_positions().transpose()<<std::endl;
        mi.render();
    }
    return 0;
}
