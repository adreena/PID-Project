#include "PID.h"
#include <math.h>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  PID::Kp = Kp;
  PID::Ki = Ki;
  PID::Kd = Kd;

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;
  cte_previous = 0.0;

  current_err = 0.0;
  best_err = INFINITY;
  dp = {1, 1, 1};
  counter = 0 ;
  bad_update_count=0;
  re_init=true;
  cte_counter = 0;
  is_twiddling = false;

//dp: 0.00271154 ,0.00221853 ,0.00246503
//K: 1 ,0 ,0.00246503

}
// double PID::SimulateMove(double steer_angle, double speed) {
//   double err =0.0f;
//   double max_angle= M_PI/4;
//   if(steer_angle > max_angle ){
//     steer_angle = max_angle;
//   }
//   else if(steer_angle < -max_angle){
//     steer_angle = -max_angle;
//   }
//
// }
//

// int PID::LastUpdatedIndex(){
//   int index= -1;
//   for(unsigned i =0; i< 3; i++){
//     if(last_update[i])
//         return i;
//   }
//   return index;
// }

void PID::UpdateError(double cte) {
  cte_counter++;
  // double total_err = TotalError();
  // if (counter > 1000)
  current_err += cte*cte;
  // bool twiddle = false;
  // std::cout<<"** counter"<<counter<<endl;
  p_error = cte;
  d_error = cte - cte_previous ;
  i_error += cte;
  double total_dp = Totaldp();
  std::cout<<"** total_dp "<<total_dp<<endl;
  std::cout<<"** current_err "<<(current_err/cte_counter)<<endl;
  std::cout<<"** cte_counter "<<cte_counter<<endl;
  std::cout<<"** error "<<p_error<<", "<<d_error<<", "<<i_error<<endl;
  // double new_err = INFINITY;
  // if (total_dp > 0.2){
  //     bool counter_reached = false;
  //     if (!is_twiddling){ 
  //       if(cte_counter == 200){
  //         best_err = current_err/200;
  //         counter_reached = true;
  //       }
  //     }
  //     else
  //     {
  //       if(cte_counter == 7){
  //         new_err = current_err/7;
  //         std::cout<<"XxXXXXXXX"<<endl;
  //         counter_reached = true;
  //       }
  //     }
  //     if(counter_reached){
  //       best_err = twiddle(new_err, best_err);
  //       is_twiddling = true;
  //       current_err = 0;
  //       cte_counter= 0;
  //       // i_error = cte;
  //     }
      
  //   }

  //first 100 : best_err 0.571984
  if(cte_counter == 10000){
     best_err = current_err/10000;
     std::cout<<"XxXXXXXXX"<<best_err<<endl;
     exit(0);
  }
  

  
  // std::cout<<"cte_previous: "<<cte_previous<<" cte: "<<cte<<" , total_dp: "<< total_dp<<endl;
  // std::cout<<"dp: "<<dp[0]<<" ,"<<dp[1]<<" ,"<< dp[2]<<endl;
  std::cout<<"K: "<<Kp<<" ,"<<Ki<<" ,"<< Kd<<endl;
  // // std::cout<<"err: "<<p_error<<" ,"<<i_error<<" ,"<< d_error<<endl;
  cte_previous = cte;
  // std::cout<<"-----"<<endl;
}

double PID::twiddle(double new_err, double best_err){
  
  // std::cout<<"** best error "<<best_err<<endl;
  int previous_index = (counter-1) %3;
  int index = counter %3;
  std::cout<<"BEFORE: "<<best_err<<", new_err:"<<new_err<<endl;
  // do the first twiddle
  if(previous_index < 0){
    UpdateParams(index,1);
    std::cout<<"p["<<index<<"] += dp["<<index<<"]"<<endl;
    counter++;
    return best_err;
  }

    if( new_err < best_err){
      std::cout<<"1"<<endl;
      bad_update_count = 0;
      best_err = new_err;
      dp[previous_index]*=1.1;
      std::cout<<"dp["<<previous_index<<"]*=1.1"<<endl;
      if (Totaldp() > 0.2){
        UpdateParams(index,1);
        std::cout<<"p["<<index<<"] += dp["<<index<<"]"<<endl;
      }
      counter++;
    }
    else{
      if ( bad_update_count <1 ){
        std::cout<<"2"<<endl;
        std::cout<<"p["<<previous_index<<"] -= 2*dp["<<previous_index<<"]"<<endl;
        UpdateParams(previous_index,-2);
        bad_update_count++;
      }
      else{
        std::cout<<"3"<<endl;
        std::cout<<"p["<<previous_index<<"] += dp["<<previous_index<<"]"<<endl;
        UpdateParams(previous_index,1);
        dp[previous_index] *=0.95;
        std::cout<<"dp["<<previous_index<<"] *=0.9"<<endl;
        counter++;
        if (Totaldp() > 0.2){
          UpdateParams(index,1);
          std::cout<<"p["<<index<<"] += dp["<<index<<"]"<<endl;
        }
        bad_update_count=0;
      }
    }
    std::cout<<"AFTER: "<<best_err<<endl;
    std::cout<<"** dp "<<dp[0]<<", "<<dp[1]<<", "<<dp[2]<<endl;
    std::cout<<"--------------------------"<<endl;
    return best_err;
}


void PID::UpdateParams(int index, int coef){
  if (index == 0){
    Kp += coef * dp[index];
  }
  else if (index == 1){
    Ki += coef * dp[index];
  }
  else if (index == 2){
    Kd += coef * dp[index];
  }
}

double PID::TotalError() {
  return Kp * p_error + Ki * i_error + Kd * d_error;
}

double PID::Totaldp() {
  return dp[0]+dp[1]+dp[2];
}
