#include "behavior_planner.h"
#include "math.h"
#include <limits>

#include <iostream>
Trajectory BehaviorPlanner::generateTrajectory(double T)
{
    //only consider states which can be reached from current FSM state.
    T_ = T;
    vector<Vstate> possible_next_states = successorStates();
    bool rerror;
    Vehicle end = generateGoal(car_.state_, rerror);
    double min_cost = std::numeric_limits<double>::max();
    double cost;
    for(Vstate next_state : possible_next_states)
    {
        Vehicle goal = generateGoal(next_state, rerror);
        if(rerror)
            continue;
        cost = behaviorCost_.calculateCost(targetSpeed_, predictions_, car_, goal);
        if(cost < min_cost)
        {
            min_cost = cost;
            end = goal;
        }
    }
    car_.state_ = end.state_;//update the state
#ifdef DEBUG
    cout << "next state: ";
    switch(car_.state_)
    {
        case CS:
            cout << "CS";
            break;
        case KL:
            cout << "KL";
            break;
        case LCL:
            cout << "LCL";
            break;
        case LCR:
            cout << "LCR";
            break;
        case PLCL:
            cout << "PLCL";
            break;
        case PLCR:
            cout << "PLCR";
            break;
    }
    cout << endl;
#endif
    return ptg_.generate(car_, end, predictions_, T);
}

void BehaviorPlanner::updateLocation(const vector<double> & loc, double speedLimit, double t)
{
    double prev_v = car_.v_;
    double prev_t = car_.t_;
    preferredBuffer_ = 6.0;//TODO: this should be based on current speed
    speedLimit_ = speedLimit;
    targetSpeed_ = speedLimit*0.80;//target speed is a fraction of the speed limit
    car_.s_ = loc[0];
    car_.d_ = loc[1];
    car_.v_ = loc[2];
    car_.lane_ = getLane(car_.d_);
    car_.t_ = t;
    if(t > 0)
    {
        car_.a_ = (car_.v_ - prev_v)/(t - prev_t);
    }
    else
    {
        car_.a_ = 0.0;
    }
    #ifdef DEBUG
    cout << "t= "<< t;
    cout << ", car.s= " << car_.s_ << ", car.d= " << car_.d_;
    cout << ", car.v= " << car_.v_ <<", car.a="<< car_.a_ << endl;
    #endif
}

void BehaviorPlanner::updatePredictions(const vector<vector<double>> & sensor_fusion, double t)
{
    // Get predictions from sensor fusion data, which is a vector of elements
    // with the following format: [id, x, y, vx, vy, s, d]
    int id;
    double vx, vy;
    double prev_v, prev_t;
    map<int, Vehicle>::iterator it;
    for(int i = 0; i < sensor_fusion.size(); ++i)
    {
        Vehicle v;
        id = (int)sensor_fusion[i][0];
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        
        v.s_ = sensor_fusion[i][5];
        v.d_ = sensor_fusion[i][6];
        v.v_ = sqrt(vx*vx + vy*vy);
        v.a_ = 0.0;
        v.t_ = t;
        v.lane_ = getLane(v.d_);
        //check if in predictions_ map already exists a Vehicle with the same id
        it = predictions_.find(id);
        if(it != predictions_.end())
        {
            prev_t = predictions_[id].t_;
            prev_v = predictions_[id].v_;
            if(t > 0)
            {
                v.a_ = (v.v_ - prev_v)/(v.t_ - prev_t);
            }
        }
        predictions_[id] = v;
    }
}

vector<Vstate> BehaviorPlanner::successorStates()
{
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<Vstate> states;
    states.push_back(KL);
    switch(car_.state_)
    {
        case KL:
            states.push_back(PLCL);
            states.push_back(PLCR);
            break;
        case PLCL:
            states.push_back(PLCL);
            states.push_back(LCL);
            break;
        case PLCR:
            states.push_back(PLCR);
            states.push_back(LCR);
    }

    //If state is "LCL" or "LCR", then just return "KL"
    return states;
}

Vehicle BehaviorPlanner::generateGoal(Vstate state, bool & rerror)
{
    /*
     *  Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    Vehicle goal;
    rerror = false;
    switch(state)
    {
        case CS:
            goal = constanSpeedTrajectory();
            break;
        case KL:
            goal = keepLaneTrajectory();
            break;
        case LCL:
        case LCR:
            goal = laneChangeTrajectory(state, rerror);
            break;
        case PLCL:
        case PLCR:
            goal = prepLaneChangeTrajectory(state);
            break;
    }
    return goal;
}

int BehaviorPlanner::getLane(double d)
{
    if(d >= 0 && d < laneWidth_*nLanes_)
    {
        int w = laneWidth_;
        int lane = 0;
        while(w < d)
        {
            w *= 2;
            lane++;
        }
        return lane;
    }
    else
    {
        return -1;//ERROR, out of road
    }
}

bool BehaviorPlanner::getVehicleBehind(int lane, Vehicle & rVehicle)
{
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int max_s = -1;
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, Vehicle>::iterator it = predictions_.begin(); it != predictions_.end(); ++it) {
        temp_vehicle = it->second;
        if (temp_vehicle.lane_ == lane && temp_vehicle.s_ < car_.s_ && temp_vehicle.s_ > max_s) {
            max_s = temp_vehicle.s_;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

bool BehaviorPlanner::getVehicleAhead(int lane, Vehicle & rVehicle)
{
    /*
    Returns a true if a vehicle is found ahead of the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    int min_s = std::numeric_limits<int>::max();
    bool found_vehicle = false;
    Vehicle temp_vehicle;
    for (map<int, Vehicle>::iterator it = predictions_.begin(); it != predictions_.end(); ++it) {
        temp_vehicle = it->second;
        if (temp_vehicle.lane_ == lane && temp_vehicle.s_ > car_.s_ && temp_vehicle.s_ < min_s) {
            min_s = temp_vehicle.s_;
            rVehicle = temp_vehicle;
            found_vehicle = true;
        }
    }
    return found_vehicle;
}

vector<double> BehaviorPlanner::getKinematics(int lane)
{
    /* 
    Gets next timestep kinematics (position, velocity, acceleration) 
    for a given lane. Tries to choose the maximum velocity and acceleration, 
    given other vehicle positions and accel/velocity constraints.
    */
    double max_velocity_accel_limit = (10.0 + car_.v_)*T_;
    double new_position;
    double new_velocity;
    double new_accel;
    Vehicle vehicleAhead;
    Vehicle vehicleBehind;
    double max_velocity_in_front = -100;
    if (getVehicleAhead(lane, vehicleAhead)) 
    {
        if (getVehicleBehind(lane, vehicleBehind)) {
            new_velocity = vehicleAhead.v_; //must travel at the speed of traffic, regardless of preferred buffer      
#ifdef DEBUG
            cout << "must travel at the speed of traffic" << endl;
#endif
        } else {
#ifdef DEBUG
            cout << "no vehicle behind" << endl;
#endif
            max_velocity_in_front = (vehicleAhead.s_ - car_.s_ - preferredBuffer_) +
                                          vehicleAhead.v_*T_ - 0.5 * car_.a_*T_*T_;
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), targetSpeed_);
#ifdef DEBUG
            cout << "max_velocity_in_front: "<< max_velocity_in_front;
            cout << ", max_velocity_accel_limit: " << max_velocity_accel_limit;
            cout << ", targetSpeed_"<<  targetSpeed_<< endl;
#endif
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, targetSpeed_);
    }
    
    new_accel = (new_velocity - car_.v_)/T_;
    new_position = car_.s_ + new_velocity*T_ + 0.5 * new_accel *T_*T_;
    if(new_position < 0)
    {
#ifdef DEBUG
        cout << "max_velocity in front = " << max_velocity_in_front<< ", ";
        cout << " new_velocity = " << new_velocity << ", new_accel = "<< new_accel << endl;
#endif
        new_position = car_.s_;
        new_velocity = 0;
        new_accel = 0;
    }
    return{new_position, new_velocity, new_accel};
    
}

Vehicle BehaviorPlanner::constanSpeedTrajectory()
{ 
    /*
     * Generate a constant speed trajectory.
     */
    double s = car_.s_;
    double d = car_.d_;
    double v = car_.v_;
    double a = car_.a_;
    int lane = car_.lane_;
    
    double next_s = car_.positionAt(T_);
    d = lane*laneWidth_ + laneWidth_/2;//fix d
    Vehicle goal = Vehicle(lane, next_s, d, v, 0, CS);
    
    return goal;
}

Vehicle BehaviorPlanner::keepLaneTrajectory()
{
    /*
     * Generate a keep lane trajectory.
     */
    double s = car_.s_;
    double d = car_.d_;
    double v = car_.v_;
    double a = car_.a_;
    int lane = car_.lane_;
    
    vector<double> kinematics = getKinematics(lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    d = lane*laneWidth_ + laneWidth_/2;//fix d
    Vehicle goal = Vehicle(lane, new_s, d, new_v, new_a, KL);
    return goal;
}

Vehicle BehaviorPlanner::laneChangeTrajectory(Vstate state, bool & rerror)
{
    /*
    Generate a lane change trajectory.
    */
    int lane = car_.lane_;
    int new_lane = lane + mapLaneDirection_[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    Vehicle goal;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, Vehicle>::iterator it = predictions_.begin(); it != predictions_.end(); ++it) {
        next_lane_vehicle = it->second;
        if (next_lane_vehicle.s_ == car_.s_ && next_lane_vehicle.lane_ == new_lane) {
            //If lane change is not possible, return empty trajectory.
            rerror = true;
            return goal;
        }
    }
    vector<double> kinematics = getKinematics(new_lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    double new_d = new_lane*laneWidth_ + laneWidth_/2;
    goal = Vehicle(new_lane, new_s, new_d, new_v, new_a, state);
    return goal;
}

Vehicle BehaviorPlanner::prepLaneChangeTrajectory(Vstate state)
{
    /*
    * Generate a trajectory preparing for a lane change.
    */
    double new_s;
    double new_v;
    double new_a;
    Vehicle vehicleBehind;
    int lane = car_.lane_;
    int new_lane = lane + mapLaneDirection_[state];
    vector<double> curr_lane_new_kinematics = getKinematics(lane);

    if (getVehicleBehind(lane, vehicleBehind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        
    } else {
        vector<double> best_kinematics;
        vector<double> next_lane_new_kinematics = getKinematics(new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
    }

    double d = car_.lane_*laneWidth_ + laneWidth_/2;//fix d
    Vehicle goal = Vehicle(lane, new_s, d, new_v, new_a, state);
    return goal;
}
