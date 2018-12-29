#include "behavior_planner.h"
#include "math.h"

vector<vector<double>> BehaviorPlanner::generateTrajectory()
{
    //only consider states which can be reached from current FSM state.
    vector<Vstate> possible_next_states = successor_states();
    vector<Vehicle> highLevelTrajectory = generateHighLevelTrajectory(KL);
    double min_cost = 99999.99;
    double cost;
    for(Vstate next_state : possible_next_states)
    {
        vector<Vehicle> trajectory = generateHighLevelTrajectory(next_state);
        if(trajectory.empty())
            continue;
        cost = calculateCost(trajectory);
        if(cost < min_cost)
        {
            min_cost = cost;
            highLevelTrajectory = trajectory;
        }
    }

    return trajectoryGenerator_.generate(highLevelTrajectory, predictions_);
}

void BehaviorPlanner::updateLocation(const vector<double> & loc)
{
    double prev_speed;
    double prev_t;
    prev_t = egoVehicle_.timestamp;
    prev_speed = egoVehicle_.speed;
    egoVehicle_.s = loc[0];
    egoVehicle_.d = loc[1];
    egoVehicle_.speed = loc[2];
    egoVehicle_.timestamp = loc[3];
    if(egoVehicle_.timestamp > 0.00001)
    {
        egoVehicle_.acc = (egoVehicle_.speed - prev_speed)/(egoVehicle_.timestamp - prev_t);
    }
    else
    {
        egoVehicle_.acc =0.0;
    }
}

void BehaviorPlanner::updatePredictions(const vector<vector<double>> & sensorFusion, double timestamp)
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
        
        v.s = sensor_fusion[i][5];
        v.d = sensor_fusion[i][6];
        v.speed = sqrt(vx*vx + vy*vy);
        v.timestamp = timestamp;
        v.acc = 0.0;
        //check if in predictions_ map already exists a Vehicle with the same id
        it = predictions_.find(id);
        if(it != predictions_.end())
        {
            predictions_[id].s = v.s;
            predictions_[id].d = v.d;
            if(v.timestamp > 0.00001)
            {
                predictions_[id].acc = (v.speed - predictions_[id].speed)/(v.timestamp - predictions_[id].timestamp);
            }
            else
            {
                predictions_[id].acc = 0.0;
            }
            predictions_[id].speed = v.speed;
            predictions_[id].timestamp = v.timestamp;
        }
        else
        {
            predictions_[id] = v;
        }
    }
}

vector<VState> BehaviorPlanner::successorStates()
{
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<VState> states;
    states.push_back(KL);
    switch(this->state_)
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

vector<Vehicle> BehaviorPlanner::generateHighLevelTrajectory(const Vstata state)
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

double BehaviourPlanner::calculateCost(const vector<Vehicle> & trajectory)
{
}

vector<Vehicle> BehaviourPlanner::constanSpeedTrajectory()
{
    /*
     * Generate a constant speed trajectory.
     */
    vector<Vehicle> trajectory;
    float next_pos = position_at(1);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->v, this->a, this->state),
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state)};
    return trajectory;
}
}

vector<Vehicle> BehaviourPlanner::keepLaneTrajectory()
{
    vector<Vehicle> trajectory;
}

vector<Vehicle> BehaviourPlanner::laneChangeTrajectory(Vstate state)
{
    vector<Vehicle> trajectory;
}

vector<Vehicle> BehaviourPlanner::prepLaneChangeTrajectory(Vstate state)
{
    vector<Vehicle> trajectory;
}
