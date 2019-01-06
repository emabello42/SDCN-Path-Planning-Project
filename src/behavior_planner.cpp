#include "behavior_planner.h"
#include "math.h"
#include "lane.h"
#include <vector>

CarCommand BehaviorPlanner::getNextAction(const Vehicle & refCar, double targetSpeed, Vstate & rNextState, double dt)
{
    /*
     * Evaluate the current information about location and from sensor fusion
     * to take the best action.
     * Arguments:
     *  - refCar: keeps the information of the egoCar (could be the last point taken from the last trajectory generated)
     *  - targetSpeed: speed limit
     *  - rNextState: used to return the next state
     *  - dt: time since last time
     * Return valur:
     *  CarCommand: commands about the steering wheel and speed:
     *      - Wheel: TURN_LEFT, TURN_RIGHT or just keep the same direction
     *      (KEEP_DIR)
     *      - Speed: SPEED_UP, SPEED_DOWN or just keep the same speed
     *      (KEEP_SPEED)
     */
    CarCommand cmd;
    Vstate bestState = refCar.state;
    vector<Lane> lanes = vector<Lane>(nLanes_);
    Vehicle tmpCar;
    bool too_close = false;

    //analyse predictions
    for(map<int, Vehicle>::iterator it = predictions_.begin(); it != predictions_.end(); ++it)
    {
        tmpCar = it->second;
        int idxLane = tmpCar.lane;
        double s = tmpCar.positionAt(dt);
        double v = tmpCar.speedAt(dt);
        double dist = fabs(s - refCar.s);//s distance from refCar
        
        //check if the refCar is too close to the car in front
        if(idxLane == refCar.lane && dist < 30 && s > refCar.s)
        {
            too_close = true;
        }

        // check the vehicle behind
        if (s < refCar.s && dist < lanes[idxLane].distBehind)
        {
            lanes[idxLane].foundVehicleBehind = true;
            lanes[idxLane].distBehind = dist;
            lanes[idxLane].vehicleBehind = tmpCar;
            lanes[idxLane].vehicleBehind.s = s;
            lanes[idxLane].vehicleBehind.v = v;
        }
        // check the vehicle ahead
        else if (s > refCar.s && dist < lanes[idxLane].distAhead)
        {
            lanes[idxLane].foundVehicleAhead = true;
            lanes[idxLane].distAhead = dist;
            lanes[idxLane].vehicleAhead = tmpCar;
            lanes[idxLane].vehicleAhead.s = s;
            lanes[idxLane].vehicleAhead.v = v;
        }
        //check if there is a car parallel to the ego car, thus avoiding
        //collisions
        if(dist < 3)
        {
            lanes[idxLane].samePostionInS = true;
        }


        //update measurements
        if(dist < 60)
        {
            //consider the speed of the cars close to the ego car only
            lanes[idxLane].noTraffic = false;
            lanes[idxLane].trafficSpeed = v;
        }
        if(s > refCar.s && dist < 90)
        {
            //considers only the traffic ahead only
            lanes[idxLane].trafficDensityAhead += 1;  
        }
    }//end for
    
    double min_cost = std::numeric_limits<double>::max();
    double cost;

    //if there is no car in fron of us, just try to accelerate until reaching
    //the speed limit (this is cheched during trajectory generation)
    if(!too_close)
    {
        cmd.speed = SPEED_UP;
        cmd.wheel = KEEP_DIR;
        rNextState = bestState;
        return cmd;
    }
    //...otherwise, take some action:

    //get possible next states and analysis the cost of each action to transit
    //to each possible state. Before calculating the cost of every action is
    //evaluated the feasability of such action
    vector<Vstate> nextStates = successorStates(refCar.state);
    for(Vstate nextState : nextStates)
    {
        int idxFinalLane;//indicates the lane approached during every state
        int idxIntendedLane;//indicates the final lane, where the car is going to be during every state
        SpeedCommand scmd = too_close ? SPEED_DOWN : SPEED_UP;//default speed command

        switch(nextState)
        {
            case KL:
                idxIntendedLane = refCar.lane;
                idxFinalLane = refCar.lane;
                break;
            
            case PLCL:
            case PLCR:
                idxIntendedLane = refCar.lane + mapLaneDirection_[nextState];
                idxFinalLane = refCar.lane;
                //check if the lane considered is valid
                if(idxIntendedLane < 0 || idxIntendedLane >= nLanes_) continue;
                
                //Try to speed up if the car is not so close to the car in
                //front, in order to have more chances to change lane
                if(lanes[refCar.lane].distAhead > 25)
                {
                    if(lanes[idxIntendedLane].foundVehicleBehind &&
                       lanes[idxIntendedLane].distBehind < 30 &&
                       refCar.v < lanes[idxIntendedLane].vehicleBehind.v)
                        scmd = SPEED_UP;
                }
                break;
            
            case LCL:
            case LCR:
                idxIntendedLane = refCar.lane + mapLaneDirection_[nextState];
                idxFinalLane = refCar.lane + mapLaneDirection_[nextState];
                if(idxIntendedLane < 0 || idxIntendedLane >= nLanes_) continue;

                if(lanes[idxFinalLane].samePostionInS) continue;//avoid collisions
                
                //avoid collision with the car behind
                if(lanes[idxFinalLane].foundVehicleBehind &&
                   lanes[idxFinalLane].distBehind < 5 &&
                   refCar.v < lanes[idxFinalLane].vehicleBehind.v)
                    continue;

                //avoid collision with the car ahead
                if(lanes[idxFinalLane].foundVehicleAhead &&
                   lanes[idxFinalLane].distAhead < 30)
                    continue;
                
                break;
        }//end switch
        cost = behaviorCost_.calculateCost(targetSpeed, refCar, lanes[idxFinalLane], lanes[idxIntendedLane]);
        if(cost < min_cost)
        {
            min_cost = cost;
            cmd.speed = scmd;
            bestState = nextState;
            if(bestState == LCL)
                cmd.wheel = TURN_LEFT;
            else if(bestState == LCR)
                cmd.wheel = TURN_RIGHT;
            else
                cmd.wheel = KEEP_DIR;
        }
    }//end for
    rNextState = bestState;
    return cmd;
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
        Vehicle car;
        id = (int)sensor_fusion[i][0];
        vx = sensor_fusion[i][3];
        vy = sensor_fusion[i][4];
        
        car.s = sensor_fusion[i][5];
        car.d = sensor_fusion[i][6];
        car.v = sqrt(vx*vx + vy*vy);
        car.a = 0.0;
        car.t = t;
        car.lane = getLane(car.d);
        if(car.lane < 0)
            continue; //ignore cars outside my road
        //check if in predictions_ map already exists a Vehicle with the same id
        it = predictions_.find(id);
        if(it != predictions_.end())
        {
            prev_t = predictions_[id].t;
            prev_v = predictions_[id].v;
            if(t > 0)
            {
                car.a = (car.v - prev_v)/(car.t - prev_t);
            }
        }
        predictions_[id] = car;
    }
}

vector<Vstate> BehaviorPlanner::successorStates(Vstate state)
{
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<Vstate> states;
    states.push_back(KL);
    switch(state)
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
