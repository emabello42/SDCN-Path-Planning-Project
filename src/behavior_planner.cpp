#include "behavior_planner.h"
#include "math.h"
#include <limits>

#include <iostream>
vector<vector<double>> BehaviorPlanner::generateTrajectory(double dt, int nPoints)
{
    //only consider states which can be reached from current FSM state.
    dt_ = dt;
    nPoints_ = nPoints;
    vector<Vstate> possible_next_states = successorStates();
    vector<Vehicle> highLevelTrajectory = generateHighLevelTrajectory(car_.state_);
    double min_cost = std::numeric_limits<double>::max();
    double cost;
    for(Vstate next_state : possible_next_states)
    {
        vector<Vehicle> trajectory = generateHighLevelTrajectory(next_state);
        if(trajectory.empty())
            continue;
        cost = behaviorCost_.calculateCost(targetSpeed_, predictions_, trajectory);
        if(cost < min_cost)
        {
            min_cost = cost;
            highLevelTrajectory = trajectory;
        }
    }
    car_.state_ = highLevelTrajectory[highLevelTrajectory.size()-1].state_;//update the state
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
    Vehicle start = highLevelTrajectory[0];
    Vehicle goal = highLevelTrajectory[highLevelTrajectory.size()-1];
    cout << ", start: s="<< start.s_ << ", d=" << start.d_ << ", v="<< start.v_ << ", a="<<start.a_; 
    cout << "| goal: s="<< goal.s_ << ", d=" << goal.d_ << ", v=" << goal.v_ << ", a=" << goal.a_; 
    cout << endl;
    return ptg_.generate(highLevelTrajectory, predictions_, dt_, nPoints_);
}

void BehaviorPlanner::updateLocation(const vector<double> & loc, double speedLimit)
{
    double prev_v;
    prev_v = car_.v_;
    preferredBuffer_ = 6.0;//TODO: this should be based on current speed
    speedLimit_ = speedLimit;
    targetSpeed_ = speedLimit*0.80;//target speed is a fraction of the speed limit
    car_.s_ = loc[0];
    car_.d_ = loc[1];
    car_.v_ = loc[2];
    car_.lane_ = getLane(car_.d_);
    cout << "car.s= " << car_.s_ << ", car.d= " << car_.d_;
    cout << ", car.v= " << car_.v_ << endl; 
}

void BehaviorPlanner::updatePredictions(const vector<vector<double>> & sensor_fusion)
{
    // Get predictions from sensor fusion data, which is a vector of elements
    // with the following format: [id, x, y, vx, vy, s, d]
    int id;
    double vx, vy;
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
        v.lane_ = getLane(v.d_);
        //check if in predictions_ map already exists a Vehicle with the same id
        it = predictions_.find(id);
        if(it != predictions_.end())
        {
            predictions_[id].s_ = v.s_;
            predictions_[id].d_ = v.d_;
            predictions_[id].lane_ = v.lane_;
            predictions_[id].v_ = v.v_;
        }
        else
        {
            predictions_[id] = v;
        }
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

vector<Vehicle> BehaviorPlanner::generateHighLevelTrajectory(Vstate state)
{
    /*
     *  Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    switch(state)
    {
        case CS:
            trajectory = constanSpeedTrajectory();
            break;
        case KL:
            trajectory = keepLaneTrajectory();
            break;
        case LCL:
        case LCR:
            trajectory = laneChangeTrajectory(state);
            break;
        case PLCL:
        case PLCR:
            trajectory = prepLaneChangeTrajectory(state);
            break;
    }
    return trajectory;
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
    double t = dt_*nPoints_;
    double max_velocity_accel_limit = (10.0 + car_.v_)*t;
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
            cout << "must travel at the speed of traffic" << endl;
        } else {
            cout << "no vehicle behind" << endl;
            max_velocity_in_front = (vehicleAhead.s_ - car_.s_ - preferredBuffer_) +
                                          vehicleAhead.v_ - 0.5 * car_.a_;
            new_velocity = min(min(max_velocity_in_front, max_velocity_accel_limit), targetSpeed_);
            cout << "max_velocity_in_front: "<< max_velocity_in_front;
            cout << ", max_velocity_accel_limit: " << max_velocity_accel_limit;
            cout << ", targetSpeed_"<<  targetSpeed_<< endl;
        }
    } else {
        new_velocity = min(max_velocity_accel_limit, targetSpeed_);
    }
    
    new_accel = (new_velocity - car_.v_)/t;
    new_position = car_.s_ + new_velocity*t + 0.5 * new_accel *t*t;
    if(new_position < 0)
    {
        cout << "max_velocity in front = " << max_velocity_in_front<< ", ";
        cout << " new_velocity = " << new_velocity << ", new_accel = "<< new_accel << endl;
        new_position = car_.s_;
        new_velocity = 0;
        new_accel = 0;
    }
    return{new_position, new_velocity, new_accel};
    
}

vector<Vehicle> BehaviorPlanner::constanSpeedTrajectory()
{ 
    /*
     * Generate a constant speed trajectory.
     */
    double s = car_.s_;
    double d = car_.d_;
    double v = car_.v_;
    double a = car_.a_;
    int lane = car_.lane_;
    vector<Vehicle> trajectory = {Vehicle(lane, s, d, v, a, car_.state_)};
    
    double next_s = car_.positionAt(dt_*nPoints_);
    d = lane*laneWidth_ + laneWidth_/2;//fix d
    trajectory.push_back(Vehicle(lane, next_s, d, v, 0.0, CS));
    
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::keepLaneTrajectory()
{
    /*
     * Generate a keep lane trajectory.
     */
    double s = car_.s_;
    double d = car_.d_;
    double v = car_.v_;
    double a = car_.a_;
    int lane = car_.lane_;
    vector<Vehicle> trajectory;
    if(lane == -1 )
        return trajectory;
    
    trajectory.push_back(Vehicle(lane, s, d, v, a, car_.state_));
    vector<double> kinematics = getKinematics(lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    d = lane*laneWidth_ + laneWidth_/2;//fix d
    trajectory.push_back(Vehicle(lane, new_s, d, new_v, new_a, KL));
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::laneChangeTrajectory(Vstate state)
{
    /*
    Generate a lane change trajectory.
    */
    int lane = car_.lane_;
    int new_lane = lane + mapLaneDirection_[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, Vehicle>::iterator it = predictions_.begin(); it != predictions_.end(); ++it) {
        next_lane_vehicle = it->second;
        if (next_lane_vehicle.s_ == car_.s_ && next_lane_vehicle.lane_ == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(car_.lane_, car_.s_, car_.d_, car_.v_, car_.a_, car_.state_));
    vector<double> kinematics = getKinematics(new_lane);
    double new_s = kinematics[0];
    double new_v = kinematics[1];
    double new_a = kinematics[2];
    double new_d = new_lane*laneWidth_ + laneWidth_/2;
    trajectory.push_back(Vehicle(new_lane, new_s, new_d, new_v, new_a, state));
    return trajectory;
}

vector<Vehicle> BehaviorPlanner::prepLaneChangeTrajectory(Vstate state)
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
    vector<Vehicle> trajectory = {Vehicle(lane, car_.s_, car_.d_, car_.v_, car_.a_, car_.state_)};
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
    trajectory.push_back(Vehicle(lane, new_s, d, new_v, new_a, state));
    return trajectory;
}
