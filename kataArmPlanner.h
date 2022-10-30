#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 
#include <iterator>
#include <algorithm>    // std::min std::max
#include <assert.h>     /* assert */
#include <math.h>       /* log */
#include <list>
#include <stdio.h>
#include <limits>
#include <queue>
#include <bits/stdc++.h>
#include <map>
#include <utility>      // std::pair, std::make_pair
#include <cmath>


class arm_state
{
    protected:
        int DOF;
        int step_num = 0; 
        std::vector<double> state;
        bool is_goal = false;
        
        int num_neighbors = 0;
        std::vector<arm_state*> connected_neighbors;

        std::vector<int> cluster_id;
        int tree_id = -1;
        double cost = 0;

    public: 
        arm_state()
        {
            //nothing
        }

        arm_state(int dof, double* anglesV_rad)
        {
            init(dof);
            for(int i = 0; i < dof; ++i)
            {
                this->state.push_back(anglesV_rad[i]);
            }   
        }

        arm_state(int DOF)
        {
            init(DOF);
        }

        arm_state(std::vector<double> input_state)
        {
            init((input_state.size()));
            for(int i = 0; i < input_state.size(); ++i)
            {
                state.push_back(input_state[i]);
            }            
        }

        arm_state(arm_state* original)
        {
            init(original->get_dof());
            this->set_state(original->get_state());

        }

        void init(int dof)
        {
            this-> DOF = dof;
            num_neighbors = 0;
            if(connected_neighbors.empty())
            {
                connected_neighbors.clear();
            }
            if(state.empty())
            {
                state.clear();
            }
        }

        int get_num_neighbors()
        {
            return num_neighbors;
        }

        int get_dof()
        {
            return DOF;
        }

        std::vector<double> get_state()
        {
            return state;
        }

        void set_state(std::vector<double> input_state)
        {
            int dof = input_state.size();
            init(dof);
            for(int i = 0; i < dof; ++i)
            {
                this->state.push_back(input_state[i]);
            }   
        }

        double get_angle(int index)
        {
            if(index<DOF)
            {
                return state[index];
            }
            std::cout << "INVAID INDEX" << std::endl;
            return -10000.0;
        }

        std::vector<arm_state*> get_connected_neighbors()
        {
            return this->connected_neighbors;
        }

        arm_state* get_neighbor(int index)
        {
            if(index<num_neighbors)
            {
                return connected_neighbors[index];
            }
            else
            {
                return this;
            }
        }

        int add_neighbor(arm_state* new_neighbor)
        {
            connected_neighbors.push_back(new_neighbor);
            ++num_neighbors;
            return num_neighbors;
        }

        bool remove_neighbor(arm_state* neighbor)
        {
            for(int i = 0; i = this->num_neighbors; ++i)
            {
                if(connected_neighbors[i] == neighbor)
                {
                    this->num_neighbors--;
                    this->connected_neighbors.erase(this->connected_neighbors.begin()+i);
                    return true;
                }
            }
            printf("Failed to find neighbor for erasing!!!!!!!\n");
            return false;
        }

        arm_state* get_parent()
        {
            for (int i = 0; i < connected_neighbors.size(); i++)
            {
                if(connected_neighbors[i]->get_step_num() == this->step_num-1)
                {
                    return connected_neighbors[i];
                }
            }
        }
        double get_cost()
        {
            return this->cost;
        }

        void set_cost(double in_cost)
        {
            this->cost = in_cost;
        }

        double get_dist(arm_state* input_state)
        {
            assert (this->DOF == input_state->DOF);

            if(this->DOF == input_state->DOF)
            {
                double sum = 0;
                for(int i = 0; i < DOF; ++i)
                {
                    sum+=std::pow((this->get_angle(i) - input_state->get_angle(i)), 2);
                }
                sum = sqrt(sum);
                return sum;
            }
            std::cout<< "INVALID DOF " <<std::endl;
            return -2.0;
        }
       
        arm_state* get_ptr()
        {
            return this;
        }

        std::vector<double> get_component_distance(arm_state* rand_state)
        {
            std::vector<double> difference; 
            for(int i = 0; i < state.size(); ++i)
            {
                difference.push_back(rand_state->get_angle(i)-this->get_angle(i));
            }
            return difference;
        }

        std::vector<double> get_component_distance(arm_state* rand_state, double divisor)
        {
            std::vector<double> difference; 
            for(int i = 0; i < state.size(); ++i)
            {
                difference.push_back((rand_state->get_angle(i)-this->get_angle(i))/divisor); //reversed this and finds goal?
            }
            return difference;
        }

        void add_unit(std::vector<double> unit_step)
        {
            for(int i = 0; i < DOF; ++i)
            {
                this->state[i] = this->state[i] + unit_step[i]; 
            }
        }
        
        void set_is_goal(bool is_goal)
        {
            this->is_goal = is_goal;
        }

        void set_is_goal()
        {
            this->is_goal = true;
        }

        bool get_is_goal()
        {
            return this->is_goal;
        }

        void print()
        {
            for(int i = 0; i < DOF; i++)
            {
                std::cout << state[i] << "  "; 
            }
            std::cout<< " " << std::endl;

        }

        int get_step_num()
        {
            return this->step_num;
        }

        void set_step_num(int input)
        {
            this->step_num = input;
        }

        void set_tree_id(int id)
        {
            this->tree_id = id; 
        }

        int get_tree_id()
        {
            return this->tree_id;
        }

        void add_cluster(int id )
        {
            this->cluster_id.push_back(id);
        }

        std::vector<int> get_clusters()
        {
            return cluster_id;
        }

        bool in_same_cluster(arm_state* input)
        {
            std::vector<int> nei_cluster = input->get_clusters();
            for(int i = 0; i < nei_cluster.size(); ++i)
            {
                if(std::find(cluster_id.begin(), cluster_id.end(),nei_cluster[i])!=cluster_id.end())
                {
                    return true;
                }
            }
            return false;
        }

        bool in_cluster(int index)
        {
            return std::find(cluster_id.begin(), cluster_id.end(),index)!=cluster_id.end();
        }

        int get_top_cluster()
        {
            return cluster_id.front();
        }
};

class base_planner
{
    protected:
        double* map;
        int DOFs;
        int x_size;
        int y_size;
        arm_state* goal_state;
        arm_state* start_state;
        double*** plan;
        int* planlength;
       
        #define PI 3.141592654
        #define LINKLENGTH_CELLS 10 //the length of each link in the arm
        #define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

    public:
        base_planner(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
            int numofDOFs, double*** plan, int* planlength)
            {
                this->map = map;
                this->x_size = x_size;
                this->y_size = y_size;
                this->start_state = new arm_state(numofDOFs, armstart_anglesV_rad);
                this->goal_state = new arm_state(numofDOFs, armgoal_anglesV_rad);
                this->DOFs = numofDOFs; 
                this->plan = plan;
                this->planlength = planlength;

                /* initialize random seed: */
                auto seed = time(NULL);
                std::cout << "Seed: " << seed << std::endl;
                std::srand(seed);
                // std::srand(1000);
            }
        
        arm_state* gen_rand_state()
        {
            if(std::rand()%9 == 1)
            {
                return goal_state;
            }
            else
            {
                std::vector<double> rand_angles;
                for(int i = 0; i < DOFs; ++i)
                {
                    rand_angles.push_back((2*PI*(rand()%1000)/1000)); //generates a number in [0,2pi), up to 3 decimal places
                }
                arm_state* generated_state = new arm_state(rand_angles);
                return generated_state;
            }
        }
        
        arm_state* gen_rand_state_no_seed()
        {

            std::vector<double> rand_angles;
            for(int i = 0; i < DOFs; ++i)
            {
                rand_angles.push_back((2*PI*(rand()%1000)/1000)); //generates a number in [0,2pi), up to 3 decimal places
            }
            arm_state* generated_state = new arm_state(rand_angles);
            return generated_state;

        }
        
        bool is_goal_state(std::vector<double> state) // pulled from equalDoubleArrays in planner.cpp
        {
            for (int i = 0; i < state.size(); ++i) {
                if (abs(state[i]-goal_state->get_angle(i)) > 1e-3) 
                {
                    return false;
                }
            }
            // std::cout<< "GOAL FOUND ______________________________" << std::endl;
            return true;
        }

        bool is_goal_state(arm_state* input_state) // pulled from equalDoubleArrays in planner.cpp
        {
            for (int i = 0; i < input_state->get_dof(); ++i) {
                if (abs(input_state->get_angle(i)-goal_state->get_angle(i)) > 1e-3) 
                {
                    return false;
                }
            }
            // std::cout<< "GOAL FOUND ______________________________" << std::endl;
            return true;
        }

        bool states_are_equal(arm_state* left, arm_state* right)
        {
            if(left->get_dof() == right->get_dof())
            {
                for (int i = 0; i < left->get_dof(); ++i)
                {
                    if (abs(right->get_angle(i)-left->get_angle(i)) > 1e-3) 
                    {
                        return false;
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        bool states_are_equal(std::vector<double> left, std::vector<double> right)
        {
            if(left.size() == right.size())
            {
                for (int i = 0; i < left.size(); ++i)
                {
                    if (abs(right[i]-left[i]) > 1e-3) 
                    {
                        return false;
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        bool states_are_equal(arm_state* left, std::vector<double> right)
        {
            if(left->get_dof() == right.size())
            {
                for (int i = 0; i < left->get_dof(); ++i)
                {
                    if (abs(right[i]-left->get_angle(i)) > 1e-3) 
                    {
                        return false;
                    }
                }
                return true;
            }
            else
            {
                return false;
            }
        }

        //===================Pulled from assignment for checking validity================================
        typedef struct {
            int X1, Y1;
            int X2, Y2;
            int Increment;
            int UsingYIndex;
            int DeltaX, DeltaY;
            int DTerm;
            int IncrE, IncrNE;
            int XIndex, YIndex;
            int Flipped;
        } bresenham_param_t;

        bool IsValidArmConfiguration(arm_state* state)
        {
            return IsValidArmConfiguration(state, DOFs,map,x_size,y_size);
        }

        bool IsValidArmConfiguration(arm_state* state, int numofDOFs, double*	map,
                    int x_size, int y_size) {
            double x0,y0,x1,y1;
            int i;
                
            //iterate through all the links starting with the base
            x1 = ((double)x_size)/2.0;
            y1 = 0;
            for(i = 0; i < numofDOFs; i++){
                //compute the corresponding line segment
                x0 = x1;
                y0 = y1;
                x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-state->get_angle(i));
                y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-state->get_angle(i));

                //check the validity of the corresponding line segment
                if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
                    return 0;
            }    
            return 1;
        }
       
        bool IsValidArmConfiguration(std::vector<double> state)
        {
            return IsValidArmConfiguration(state, DOFs,map,x_size,y_size);
        }

        bool IsValidArmConfiguration(std::vector<double> state, int numofDOFs, double*	map,
                    int x_size, int y_size) {
            double x0,y0,x1,y1;
            int i;
                
            //iterate through all the links starting with the base
            x1 = ((double)x_size)/2.0;
            y1 = 0;
            for(i = 0; i < numofDOFs; i++){
                //compute the corresponding line segment
                x0 = x1;
                y0 = y1;
                x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-state[i]);
                y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-state[i]);

                //check the validity of the corresponding line segment
                if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
                    return 0;
            }    
            return 1;
        }

        bool IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
            bresenham_param_t params;
            int nX, nY; 
            short unsigned int nX0, nY0, nX1, nY1;

            //printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
                
            //make sure the line segment is inside the environment
            if(x0 < 0 || x0 >= x_size ||
                x1 < 0 || x1 >= x_size ||
                y0 < 0 || y0 >= y_size ||
                y1 < 0 || y1 >= y_size)
                return 0;

            ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
            ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

            //printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

            //iterate through the points on the segment
            get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
            do {
                get_current_point(&params, &nX, &nY);
                if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
                    return 0;
            } while (get_next_point(&params));

            return 1;
        }

        void get_current_point(bresenham_param_t *params, int *x, int *y) {
            if (params->UsingYIndex) {
                *y = params->XIndex;
                *x = params->YIndex;
                if (params->Flipped)
                    *x = -*x;
            }
            else {
                *x = params->XIndex;
                *y = params->YIndex;
                if (params->Flipped)
                    *y = -*y;
            }
        }

        int get_next_point(bresenham_param_t *params) {
            if (params->XIndex == params->X2) {
                return 0;
            }
            params->XIndex += params->Increment;
            if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
                params->DTerm += params->IncrE;
            else {
                params->DTerm += params->IncrNE;
                params->YIndex += params->Increment;
            }
            return 1;
        }
        void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
            params->UsingYIndex = 0;

            if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
                (params->UsingYIndex)++;

            if (params->UsingYIndex)
                {
                    params->Y1=p1x;
                    params->X1=p1y;
                    params->Y2=p2x;
                    params->X2=p2y;
                }
            else
                {
                    params->X1=p1x;
                    params->Y1=p1y;
                    params->X2=p2x;
                    params->Y2=p2y;
                }

            if ((p2x - p1x) * (p2y - p1y) < 0)
                {
                    params->Flipped = 1;
                    params->Y1 = -params->Y1;
                    params->Y2 = -params->Y2;
                }
            else
                params->Flipped = 0;

            if (params->X2 > params->X1)
                params->Increment = 1;
            else
                params->Increment = -1;

            params->DeltaX=params->X2-params->X1;
            params->DeltaY=params->Y2-params->Y1;

            params->IncrE=2*params->DeltaY*params->Increment;
            params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
            params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

            params->XIndex = params->X1;
            params->YIndex = params->Y1;
        }

        void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
            double cellsize = 1.0;
            //take the nearest cell
            *pX = (int)(x/(double)(cellsize));
            if( x < 0) *pX = 0;
            if( *pX >= x_size) *pX = x_size-1;

            *pY = (int)(y/(double)(cellsize));
            if( y < 0) *pY = 0;
            if( *pY >= y_size) *pY = y_size-1;
        }

};

class rrt:public base_planner
{
    protected:
        double epsillon = 01.2;
        int steps = 7; //number of intermediate steps along epsillon that are checked for collision
        std::vector<arm_state*> tree;
        bool goal_found = false;
        double min_dist = 100000;
        std::vector<arm_state*> plan;
        int max_iterations = 500000;

    public:
        rrt(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
            int numofDOFs, double*** plan, int* planlength):
            base_planner(map, x_size, y_size,armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength)
            {
                tree.push_back(this->start_state);
            }

        void build_rrt()
        {
            std::cout << "\t Target state:  ";
            goal_state->print();

            for(int i = 0; i < max_iterations; i++)
            {
                if(i%10000 == 0)
                {
                    std::cout << "expanding tree iteration " << i << std::endl;
                }
               
                extend_tree(gen_rand_state());
               
                if(i%10000 == 0)
                {
                    std::cout << "\t new tree size " << tree.size() << std::endl;
                    std::cout << "\tMinimum distance to goal so far: " << min_dist << std::endl;
                    std::cout << "\t Last added state:  ";
                    tree.back()->print();
                }
                if(goal_found)
                {
                    std::cout << "\t\tGoal found! Iteration: "<< i << " step number: " << tree.back()->get_step_num() << " tree size: " << tree.size() <<"\n" <<std::endl;
                    break;
                }
            }
            
            plan = generate_plan(tree.back());

            std::cout << "\nStart state: ";
            start_state->print();
            // for(int i = plan.size()-1; i >= 0; --i)
            // {
            //     plan[i]->print(); //needs to be reversed because is goal to start.
            // // }
            std::cout << "End state: ";
            goal_state->print();


        } 

        std::vector<arm_state*> get_plan()
        {
            return plan;
        }

        void extend_tree(arm_state* rand_input)
        {
            arm_state* nearest = find_nearest(rand_input);
            std::vector<double> unit_step = get_unit_step(nearest,rand_input);
            
            std::vector<double> nearest_state = nearest->get_state(); 
            std::vector<double> last_valid = nearest->get_state();
            std::vector<double> temp_state = nearest->get_state();
            
            // std::cout << "Random state:" << std::endl;
            // std::cout << "\t" ;
            // for(int i = 0; i < rand_input->get_dof(); ++i)
            // {
            //     std::cout << rand_input->get_angle(i) << " ";
            // }
            // std::cout << "  " << std::endl;
            // std::cout << "nearest state:" << std::endl;
            // std::cout << "\t" ;
            // for(int i = 0; i < nearest->get_dof(); ++i)
            // {
            //     std::cout << nearest->get_angle(i) << " ";
            // }
            // std::cout << "  " << std::endl;
            // std::cout << "unit step: " << std::endl;
            // std::cout << "\t" ;
            // for(int i = 0; i < unit_step.size(); ++i)
            // {
            //     std::cout << unit_step[i] << " ";
            // }
            // std::cout << "\n-------------------------" << std::endl;


            for(int i = 0; i < steps; ++i)
            {
                // std::transform (temp_state.begin(), temp_state.end(), unit_step.begin(), temp_state.begin(), std::plus<double>());    //check for collision
                
                for(int i = 0; i < temp_state.size(); ++i)
                {
                    temp_state[i] += unit_step[i];
                }

                if(IsValidArmConfiguration(temp_state))
                {
                    // std::cout << "\t\tThis is a valid config. \n" <<std::endl;
                    last_valid = temp_state;
                    // last_valid.assign(temp_state.begin(), temp_state.end()); //overwrites last_valid with temp_state
                    
                    if(is_goal_state(last_valid))
                    {
                        std::cout<< "1 GOAL FOUND ______________________________" << std::endl;

                        break;
                    }
                }
                else
                {
                    // std::cout << "\t\tNOT VALID \n" <<std::endl;
                    break;
                }
            }
            
            if(!std::equal(nearest_state.begin(), nearest_state.end(), last_valid.begin()) )//meaning that we could successfully move
            {
                arm_state* new_state = new arm_state(last_valid);

                min_dist = std::min(min_dist,goal_state->get_dist(new_state));

                new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                nearest->add_neighbor(new_state);
                new_state->set_step_num(nearest->get_step_num()+1);

                if(is_goal_state(new_state))
                {
                    new_state->set_is_goal();
                    goal_found = true;
                }

                tree.push_back(new_state);

            }

        }

        std::vector<double> get_unit_step(arm_state* start_state, arm_state* rand_input)
        {
            double distance = start_state->get_dist(rand_input);
            // std::cout <<"\tDistance: " << distance << " Epsillon: " << epsillon << " steps: " << steps << std::endl;
            std::vector<double> unit_step = start_state->get_component_distance(rand_input);

            // std::cout<< "Initial" << "\t" << "Scaled down" << std::endl;

            if(distance > epsillon) //check to directly add the generated state if smaller than epsillon

            {
                for(int i = 0; i < unit_step.size(); ++i)
                {
                    // std::cout<<  unit_step[i] << "\t" << unit_step[i]*(epsillon/distance)/steps << std::endl;
                    unit_step[i]=(unit_step[i]*(epsillon/distance))/steps; // divide each unit by this value to get one step of epsillon. further divide into the substeps on the way to epsillon.
                }
            }
            else
            {
                for(int i = 0; i < unit_step.size(); ++i)
                {
                    unit_step[i]=unit_step[i]/steps; // since the distance is already smaller than epsillon, just divide by steps
                }
            }

            return unit_step;
        }

        arm_state* find_nearest(arm_state* input)
        {
            int nearest_index = 0;
            double nearest_dist = 10000;

            for(int i = 0; i < tree.size(); ++i)
            {
                double cur_dist = tree[i]->get_dist(input);
                if(cur_dist < nearest_dist)
                {
                    nearest_dist = cur_dist;
                    nearest_index = i;
                }
            }
            return tree[nearest_index];
        }

        std::vector<arm_state*> generate_plan(arm_state* goal)
        {
            std::vector<arm_state*> plan;

            arm_state* current = goal; //the node that will be added to the plan
            arm_state* temp_neighbor; //the potential neighbor that is being evaluated

            printf("Goal step number: %d\n", goal->get_step_num());
            current->print();

            for(int i = 0; i < goal->get_step_num(); ++i) //iterating over the size of the plan
            {
                // printf("Iteration :%d \n", i);
                for(int j = 0; j < current->get_num_neighbors(); ++j) //iterating over all potential neighbors
                {
                    // printf("\tNeighbor number %d\n",j);
                    temp_neighbor = current->get_neighbor(j);
                    if(current->get_step_num()-1 == temp_neighbor->get_step_num()) //neighbor is going closer to the start if the step number decreases by 1
                    {
                        // printf("\tFound!\n");
                        // plan.push_back(current);
                        plan.insert(plan.begin(), current);
                        current = temp_neighbor;
                        current->print();
                        break;
                    }
                    // printf("%s:%d\n", __FUNCTION__,__LINE__);
                }
                // printf("%s:%d\n", __FUNCTION__,__LINE__);
            }
            // printf("%s:%d\n", __FUNCTION__,__LINE__);

            // printf("last push. ");
            // plan.push_back(current); // adds the start pose, which isn't strictly necessary.
            plan.insert(plan.begin(), current);
            
            // printf("done. \n");
            return plan;
        }

};

class rrt_connect:public base_planner
{
    protected:
        double epsillon = 1.2;
        int steps = 10; //number of intermediate steps along epsillon that are checked for collision
        std::vector<arm_state*> start_tree;
        std::vector<arm_state*> goal_tree;
        double min_dist = 1000000;
        int max_iterations = 100000;
        std::vector<arm_state*> plan;

        #define GOAL_TREE 1
        #define START_TREE 0

        #define TRAPPED -1
        #define REACHED -2
        #define ADVANCED -3

    public:
        rrt_connect(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
            int numofDOFs, double*** plan, int* planlength):
            base_planner(map, x_size, y_size,armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength)
            {
                start_tree.push_back(this->start_state);
                // start_state->print();
                goal_tree.push_back(this->goal_state);
                // goal_state->print();
            }


        int switch_tree_id(int tree_id) //gives the ID of the other tree. Does not update the value in tree_id
        {
            if(GOAL_TREE == tree_id)
            {
                return START_TREE;
            }
            return GOAL_TREE;
        }
        

        arm_state* get_from_other_tree(arm_state* input, int this_tree_id)
        {
            if(GOAL_TREE == switch_tree_id(this_tree_id))
            {
                // printf("Searching goal tree!\n");
                for(int i = goal_tree.size()-1; i >= 0; i--)
                {
                    if(states_are_equal(goal_tree[i],input))
                    {
                        // printf("\tFound it! %d\n", goal_tree[i]->get_step_num());
                        return goal_tree[i];
                    }
                }
            }
            else
            {
                // printf("Searching start tree!\n");
                for(int i = start_tree.size()-1; i >= 0; i--)
                {
                    if(states_are_equal(start_tree[i],input))
                    {
                        // printf("\tFound it! %d\n", start_tree[i]->get_step_num());
                        return start_tree[i];
                    }
                    else
                    {
                        // printf("Keep looking.");
                    }
                }
            }
            printf("\tFailed to find equivalent node on tree %d!!!!!!!!!!!!!!!\n", switch_tree_id(this_tree_id));
            return input;
        } 


        void run_rrt_connect_planner()
        {
            // print_trees();
            int tree_id = START_TREE;
            for(int i = 0; i < max_iterations; ++i)
            {
                // printf("Iteration # %d, generating random state and extending %d.\n", i,tree_id);
                if(TRAPPED != extend_tree(gen_rand_state_no_seed(),tree_id)) //if not trapped after extending
                {
                    // printf("\trandom extension not trapped. attempting to connect to other tree.\n");
                    // print_trees();

                    if(REACHED == connect(get_last_extended(tree_id),(tree_id)) )//extend other tree to aim at the newly added node to this tree.
                    // if(false)
                    {

                        print_trees();
                        printf("\nGoal tree %d, start tree %d\n", GOAL_TREE, START_TREE);
                        printf("This is tree %d\n\n", tree_id);

                        printf("Connection from tree %d at: ", tree_id);
                        arm_state* connect_state_tree_A = get_last_extended(tree_id);
                        connect_state_tree_A->print();
                        print_neighbors(connect_state_tree_A);

                        printf("\nEquivalent in tree %d at: ", switch_tree_id(tree_id));
                        get_from_other_tree(connect_state_tree_A,tree_id)->print();
                        print_neighbors(get_from_other_tree(connect_state_tree_A,tree_id));


                        printf("\n\nGenerating plan A (Tree %d)\n",tree_id);
                        std::vector<arm_state*> planA = generate_plan(connect_state_tree_A); //generating plan from tree A
                        print_plan(planA);
                        
                        printf("\n\nGenerating plan B (Tree %d)\n",switch_tree_id(tree_id));
                        // arm_state* same_state_other_tree = get_from_other_tree(get_last_extended(tree_id),tree_id);
                        std::vector<arm_state*> planB = generate_plan(get_from_other_tree(connect_state_tree_A,tree_id)); //generating plan from tree B
                        print_plan(planB);



                        if(GOAL_TREE == tree_id) //means that plan A is from goal_tree and plan B is from start_tree 
                        {
                            for(int i = 0; i <planB.size(); i++)  //start tree
                            {
                                plan.push_back(planB[i]);
                            }
                            for(int i = planA.size()-2; i >=0; i--) //goal tree. Subbtract 2 instead of 1 to avoid repeat of connect state
                            {
                                plan.push_back(planA[i]);
                            }
                        }
                        else
                        {
                            for(int i = 0; i <planA.size(); i++)  //start tree
                            {
                                plan.push_back(planA[i]);
                            }
                            for(int i = planB.size()-2; i >=0; i--) //goal tree. Subbtract 2 instead of 1 to avoid repeat of connect state
                            {
                                plan.push_back(planB[i]);
                            }
                        }

                        printf("Final Plan:\n");
                        print_plan(plan);

                        return;
                    }
                    else
                    {
                        // printf("\tConnection failed. \n");
                    }
                }
                else
                {
                    // printf("\trandom extension was trapped. Not attempting to connect.\n");
                }
                // printf("\tswitching tree.\n");
                tree_id = switch_tree_id(tree_id);
            }
            printf("Ran out of iterations\n");

            
        }

        std::vector<arm_state*> generate_plan(arm_state* last_extended)
        {
            std::vector<arm_state*> ret_plan;

            arm_state* current = last_extended; //the node that will be added to the plan
            arm_state* temp_neighbor; //the potential neighbor that is being evaluated

            // printf("Goal step number: %d\n", last_extended->get_step_num());
            for(int i = 0; i < last_extended->get_step_num(); ++i) //iterating over the size of the plan
            {
                // printf("Iteration :%d \n", i);
                for(int j = 0; j < current->get_num_neighbors(); ++j) //iterating over all potential neighbors
                {
                    // printf("\tNeighbor number %d\n",j);
                    temp_neighbor = current->get_neighbor(j);
                    if(current->get_step_num()-1 == temp_neighbor->get_step_num()) //neighbor is going closer to the start if the step number decreases by 1
                    {
                        // printf("\tFound!\n");
                        ret_plan.insert(ret_plan.begin(), current);
                        current = temp_neighbor;
                        // current->print();
                        break;
                    }
                    // printf("%s:%d\n", __FUNCTION__,__LINE__);
                }
                // printf("%s:%d\n", __FUNCTION__,__LINE__);
            }
            // printf("%s:%d\n", __FUNCTION__,__LINE__);

            ret_plan.insert(ret_plan.begin(), current);
            // printf("done. \n");
            return ret_plan;
        }

        int connect(arm_state* last_extended, int tree_id) //id and last extended state of THIS tree
        {
            // printf("\tCONNECT FUNCTION: INPUT TREE %d\n",tree_id);
            int cur_status = ADVANCED;

            printf("Extending tree %d to %d\n", switch_tree_id(tree_id),tree_id);
            cur_status = extent_tree_continuously(last_extended,tree_id);

            return cur_status;
        }

        // int extend_tree(arm_state* rand_input, int tree_id, bool called_from_connect) 
        // {
        //     // printf("%s:%d\n",__FUNCTION__,__LINE__);
        //     arm_state* nearest = find_nearest(rand_input, tree_id);
        //     // printf("%s:%d\n",__FUNCTION__,__LINE__);
        //     std::vector<double> unit_step = get_unit_step(nearest,rand_input);
        //     // printf("%s:%d\n",__FUNCTION__,__LINE__);
        //     std::vector<double> nearest_state = nearest->get_state(); 
        //     std::vector<double> last_valid = nearest->get_state();
        //     std::vector<double> temp_state = nearest->get_state();
        //     int status = ADVANCED;
        //     // std::cout << "\tCurrent location: ";
        //     // rand_input->print();
        //     // std::cout << "\tgoal location: ";
        //     // nearest->print();
        //     for(int i = 0; i < steps; ++i)
        //     {              
        //         // printf("\t\tStepped state: ");
        //         for(int i = 0; i < temp_state.size(); ++i)
        //         {
        //             temp_state[i] += unit_step[i];
        //             // printf("%f ",temp_state[i]);
        //         }
        //         // printf("\n");
        //         if(IsValidArmConfiguration(temp_state))
        //         {
        //             last_valid = temp_state;          
        //             if(states_are_equal(rand_input, last_valid)) //change to compare states.
        //             {
        //                 status = REACHED;
        //                 break;
        //             }
        //         }
        //         else
        //         {
        //             status = TRAPPED;
        //             break;
        //         }
        //     }
        //     if(!std::equal(nearest_state.begin(), nearest_state.end(), last_valid.begin()) )//meaning that we could successfully move
        //     {
        //         printf("neighbor from tree %d\n", tree_id);
        //         arm_state* new_state = new arm_state(last_valid);
        //         printf("\tNearest neighbor: ");
        //         nearest->print();
        //         printf("\tLast valid state: ");
        //         new_state->print();
        //         printf("\ttarget state: ");
        //         rand_input->print();          
        //         min_dist = std::min(min_dist,goal_state->get_dist(new_state));
        //         // new_state->add_neighbor(nearest); //adding each other as mutual neighbors
        //         // nearest->add_neighbor(new_state);
        //         if(called_from_connect)
        //         {
        //             arm_state* last_added_same_tree = get_last_extended((tree_id)); 
        //             new_state->add_neighbor(last_added_same_tree); //adding each other as mutual neighbors
        //             last_added_same_tree->add_neighbor(new_state);
        //             new_state->set_step_num(last_added_same_tree->get_step_num()+1);         
        //             // if(new_state->get_dist(last_added_same_tree) > epsillon)
        //             if(true)
        //             {             
        //                 printf("\t(original tree is %d\n", switch_tree_id(tree_id));
        //                 printf("\tAttaching to : ");
        //                 last_added_same_tree->print();
        //                 printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(last_added_same_tree) );
        //                 printf("\tback of tree(this?) %d ", tree_id);
        //                 get_last_extended((tree_id))->print();             
        //                 // print_tree(switch_tree_id(tree_id));
        //                 printf("\tback of tree(other?) %d ", switch_tree_id(tree_id));
        //                 get_last_extended(switch_tree_id(tree_id))->print();
        //                 // print_tree(tree_id);
        //                 printf("================\n");
        //             }          
        //         }
        //         else
        //         {
        //             new_state->add_neighbor(nearest); //adding each other as mutual neighbors
        //             nearest->add_neighbor(new_state);
        //             new_state->set_step_num(nearest->get_step_num()+1);
        //             if(new_state->get_dist(nearest) > epsillon)
        //             {       
        //                 printf("\t(original tree is %d\n", (tree_id));
        //                 printf("\tAttaching to : ");
        //                 nearest->print();
        //                 printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(nearest) );
        //             }
        //         }
        //         if(GOAL_TREE == tree_id)
        //         {
        //             goal_tree.push_back(new_state);
        //         }
        //         else
        //         {
        //             start_tree.push_back(new_state);
        //         }
        //     }
        //     return status;
        // }

        int extent_tree_continuously(arm_state* target_this_tree, int this_tree_id) //tree id is the other tree's id 
        {
            arm_state* last_extended_state = find_nearest(target_this_tree,switch_tree_id(this_tree_id)); //this will need to be updated at the end of the loop
            int status = ADVANCED;
            printf("Goal size before connect %d start size %d\n", goal_tree.size(), start_tree.size());

            while(ADVANCED == status)
            {

                std::vector<double> unit_step = get_unit_step(last_extended_state, target_this_tree);

                std::vector<double> nearest_state = last_extended_state->get_state(); // a reference to see if we can extend
                std::vector<double> last_valid = last_extended_state->get_state(); //updated continuously with the last valid configuration
                std::vector<double> temp_state = last_extended_state->get_state();                 

                // std::cout << "\tCurrent location: ";
                // rand_input->print();

                // std::cout << "\tgoal location: ";
                // nearest->print();


                for(int i = 0; i < steps; ++i) //iterating over steps
                {              
                    // printf("\t\tStepped state: ");
                    for(int i = 0; i < temp_state.size(); ++i) //iterating over angles
                    {
                        temp_state[i] += unit_step[i];
                        // printf("%f ",temp_state[i]);
                    }
                    // printf("\n");

                    if(IsValidArmConfiguration(temp_state))
                    {
                        last_valid = temp_state;
                        
                        if(states_are_equal(target_this_tree, last_valid)) //seeing if the target was reached.
                        {
                            
                            printf("\tReached at (%d)", target_this_tree->get_tree_id());
                            target_this_tree->print();
                            status = REACHED;
                            break;
                        }
                    }
                    else
                    {
                        printf("\ttrapped\n");
                        status = TRAPPED;
                        break;
                    }
                }
                
                if(!std::equal(nearest_state.begin(), nearest_state.end(), last_valid.begin()) )//meaning that we could successfully move
                {
                    //check if you hit the target. If not, check if you hit an obstacle. If not, go again.
                    arm_state* new_state = new arm_state(last_valid);
                    new_state->set_tree_id(this_tree_id);

                    new_state->add_neighbor(last_extended_state); //adding each other as mutual neighbors
                    last_extended_state->add_neighbor(new_state);
                    new_state->set_step_num(last_extended_state->get_step_num()+1);
                
                    if(new_state->get_dist(last_extended_state) > epsillon)
                    {       
                        printf("!______________________________________________________________\n");
                        printf("\t(original tree is %d\n", (target_this_tree));
                        printf("\tAttaching to : ");
                        last_extended_state->print();
                        printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(last_extended_state) );
                    }
                    if(GOAL_TREE == switch_tree_id(this_tree_id)) //add state to the appropriate tree.
                    {
                        goal_tree.push_back(new_state);
                        printf("Adding to goal tree\n");

                    }
                    else
                    {
                        start_tree.push_back(new_state);
                        printf("Adding to start tree\n");

                    }
                    

                    if(ADVANCED == status)// meaning that the tree was extended and did not run into anything. otherwise it will leave the loop anyway
                    {
                        //update last_extended state and go again.
                        printf("\tstill advanced. going agian\n");
                        last_extended_state = new_state;
                    }
                    // printf("neighbor from tree %d\n", tree_id);
                    // printf("\tNearest neighbor: ");
                    // nearest->print();

                    // printf("\tLast valid state: ");
                    // new_state->print();

                    // printf("\ttarget state: ");
                    // rand_input->print();          


                    min_dist = std::min(min_dist,goal_state->get_dist(new_state));


                    // new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                    // nearest->add_neighbor(new_state);
        
                }
            }
            // print_trees();
            printf("Goal size after connect %d start size %d\n", goal_tree.size(), start_tree.size());
            return status;
        }


        int extend_tree(arm_state* rand_input, int tree_id) 
        {
            // printf("%s:%d\n",__FUNCTION__,__LINE__);
            arm_state* nearest = find_nearest(rand_input, tree_id);
            // printf("%s:%d\n",__FUNCTION__,__LINE__);
            std::vector<double> unit_step = get_unit_step(nearest,rand_input);
            // printf("%s:%d\n",__FUNCTION__,__LINE__);

            std::vector<double> nearest_state = nearest->get_state(); 
            std::vector<double> last_valid = nearest->get_state();
            std::vector<double> temp_state = nearest->get_state();

            int status = ADVANCED;

            // std::cout << "\tCurrent location: ";
            // rand_input->print();

            // std::cout << "\tgoal location: ";
            // nearest->print();


            for(int i = 0; i < steps; ++i)
            {              
                // printf("\t\tStepped state: ");
                for(int i = 0; i < temp_state.size(); ++i)
                {
                    temp_state[i] += unit_step[i];
                    // printf("%f ",temp_state[i]);
                }
                // printf("\n");

                if(IsValidArmConfiguration(temp_state))
                {
                    last_valid = temp_state;
                    
                    if(states_are_equal(rand_input, last_valid)) //change to compare states.
                    {
                        status = REACHED;
                        break;
                    }
                }
                else
                {
                    status = TRAPPED;
                    break;
                }
            }
            
            if(!std::equal(nearest_state.begin(), nearest_state.end(), last_valid.begin()) )//meaning that we could successfully move
            {
                // printf("Tree extended\n");

                // printf("neighbor from tree %d\n", tree_id);
                arm_state* new_state = new arm_state(last_valid);
                // printf("\tNearest neighbor: ");
                // nearest->print();

                // printf("\tLast valid state: ");
                // new_state->print();

                // printf("\ttarget state: ");
                // rand_input->print();          


                min_dist = std::min(min_dist,goal_state->get_dist(new_state));


                // new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                // nearest->add_neighbor(new_state);
    
                new_state->set_tree_id(tree_id);
                new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                nearest->add_neighbor(new_state);
                new_state->set_step_num(nearest->get_step_num()+1);

                if(false)
                {       
                    printf("\tconnecting to tree %d\n", (tree_id));
                    printf("\tAttaching to : ");
                    nearest->print();
                    printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(nearest) );
                }

                if(GOAL_TREE == tree_id)
                {
                    goal_tree.push_back(new_state);
                }
                else
                {
                    start_tree.push_back(new_state);
                }
            }
            return status;
        }

        void print_neighbors(arm_state* input)
        {
            printf("%d Neighbors of (step %d) ",input->get_num_neighbors(), input->get_step_num());
            input->print();
            for(int i = 0; i < input->get_num_neighbors(); i++)
            {
                printf("\t (step %d) ", input->get_neighbor(i)->get_step_num());
                input->get_neighbor(i)->print();
            }
        }

        std::vector<double> get_unit_step(arm_state* start_state, arm_state* rand_input)
        {
            double distance = start_state->get_dist(rand_input);
            // std::cout <<"\tDistance: " << distance << " Epsillon: " << epsillon << " steps: " << steps << std::endl;
            std::vector<double> unit_step = start_state->get_component_distance(rand_input);

            // std::cout<< "Initial" << "\t" << "Scaled down" << std::endl;

            if(distance > epsillon) //check to directly add the generated state if smaller than epsillon

            {
                for(int i = 0; i < unit_step.size(); ++i)
                {
                    // std::cout<<  unit_step[i] << "\t" << unit_step[i]*(epsillon/distance)/steps << std::endl;
                    unit_step[i]=(unit_step[i]*(epsillon/distance))/steps; // divide each unit by this value to get one step of epsillon. further divide into the substeps on the way to epsillon.
                }
            }
            else
            {
                for(int i = 0; i < unit_step.size(); ++i)
                {
                    unit_step[i]=unit_step[i]/steps; // since the distance is already smaller than epsillon, just divide by steps
                }
            }

            return unit_step;
        }

        std::vector<arm_state*> get_plan()
        {
            return plan;
        }

        arm_state* find_nearest(arm_state* input, int tree_ID)
        {
            int nearest_index = 0;
            double nearest_dist = 10000;

        
            if(GOAL_TREE == tree_ID)
            {
                for(int i = goal_tree.size()-1; i >=0 ; --i)
                {
                    double cur_dist = goal_tree[i]->get_dist(input);
                    if(cur_dist < nearest_dist)
                    {
                        nearest_dist = cur_dist;
                        nearest_index = i;
                    }
                }
                return goal_tree[nearest_index];
            }
            else //START_TREE
            {
                // printf("%s:%d\n",__FUNCTION__,__LINE__);
                for(int i = start_tree.size()-1; i >=0 ; --i)
                {
                    // printf("%s:%d\n",__FUNCTION__,__LINE__);
                    // input->print();
                    // start_tree[i]->print();
                    double cur_dist = start_tree[i]->get_dist(input);
                    // printf("%s:%d\n",__FUNCTION__,__LINE__);
                    if(cur_dist < nearest_dist)
                    {
                        // printf("%s:%d\n",__FUNCTION__,__LINE__);
                        nearest_dist = cur_dist;
                        nearest_index = i;
                    }
                }
                // printf("%s:%d\n",__FUNCTION__,__LINE__);
                return start_tree[nearest_index];
            }
         
           
        }
        
        void print_tree(int tree_id)
        {
            if(GOAL_TREE == tree_id)
            {
                printf("printing goal tree: (%d) \n", GOAL_TREE);
                for(int i = 0; i <goal_tree.size(); ++i)
                {
                    printf("\t (%d)", goal_tree[i]->get_step_num());
                    goal_tree[i]->print();
                }
            }
            else
            {
                printf("printing start tree:(%d) \n", START_TREE);
                for(int i = 0; i <start_tree.size(); ++i)
                {
                    printf("\t (%d)", start_tree[i]->get_step_num());
                    start_tree[i]->print();
                }
            }
            printf("---------------- end of tree\n");

        }

        void print_trees()
        {
            printf("Tree status:\n");
            print_tree(START_TREE);
            print_tree(GOAL_TREE);
        }

        arm_state* get_last_extended(int tree_id)
            {
                if(GOAL_TREE == tree_id)
                {
                    // printf("Returning the back of the goal tree %d\n", GOAL_TREE);
                    return goal_tree.back();
                }
                // printf("Returning the back of the start tree %d\n", START_TREE);
                return start_tree.back();
            }

        void print_plan(std::vector<arm_state*> plan)
        {
            for(int i = 0; i < plan.size(); i++)
            {
                printf("\t (%d)", plan[i]->get_step_num());
                plan[i]->print();
            }
        }
};

class rrt_star:public rrt
{
    protected:
        double gamma = 1;

    public:
        rrt_star(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                int numofDOFs, double*** plan, int* planlength):
                rrt(map, x_size, y_size,armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength)
                {
                    //nothing
                }

        double calculate_radius()
        {
            int V = tree.size();
            double radius_option = std::pow((gamma*log(V)/V),double(1/tree.back()->get_dof()));
            return std::min(radius_option,epsillon);
        }


        void extend_tree(arm_state* rand_input)
        {
            arm_state* nearest = find_nearest(rand_input);
            std::vector<double> unit_step = get_unit_step(nearest,rand_input);
            
            std::vector<double> nearest_state = nearest->get_state(); 
            std::vector<double> last_valid = nearest->get_state();
            std::vector<double> temp_state = nearest->get_state();

            for(int i = 0; i < steps; ++i)
            {
                // std::transform (temp_state.begin(), temp_state.end(), unit_step.begin(), temp_state.begin(), std::plus<double>());    //check for collision
                
                for(int i = 0; i < temp_state.size(); ++i)
                {
                    temp_state[i] += unit_step[i];
                }

                if(IsValidArmConfiguration(temp_state))
                {
                    // std::cout << "\t\tThis is a valid config. \n" <<std::endl;
                    last_valid = temp_state;
                    // last_valid.assign(temp_state.begin(), temp_state.end()); //overwrites last_valid with temp_state
                    
                    if(is_goal_state(last_valid))
                    {
                        std::cout<< "1 GOAL FOUND ______________________________" << std::endl;

                        break;
                    }
                }
                else
                {
                    // std::cout << "\t\tNOT VALID \n" <<std::endl;
                    break;
                }
            }
            
            if(!std::equal(nearest_state.begin(), nearest_state.end(), last_valid.begin()) )//meaning that we could successfully move
            {
                arm_state* new_state = new arm_state(last_valid);

                min_dist = std::min(min_dist,goal_state->get_dist(new_state));

                //RRT*. This is where you decide who to connect the new state to
                std::vector<arm_state*> region_neighbors = neighbors_in_r(calculate_radius(),new_state);
                arm_state* new_parent = get_cheapest_parent(new_state,region_neighbors);

                new_state->add_neighbor(new_parent); //adding each other as mutual neighbors
                new_parent->add_neighbor(new_state);
                new_state->set_step_num(new_parent->get_step_num()+1);
                new_state->set_cost(new_parent->get_cost()+new_state->get_dist(new_parent));

                //RRT*. This is where you rewire the set of neighbors you have just evaluated.
                rewire(new_state,region_neighbors);

                if(is_goal_state(new_state))
                {
                    new_state->set_is_goal();
                    goal_found = true;
                }

                tree.push_back(new_state);

            }

        }

        std::vector<arm_state*> neighbors_in_r(double radius, arm_state* input)
        {
            std::vector<arm_state*> neighbors; 
            for(int i = 0; i < tree.size(); ++i)
            {
                double cur_dist = tree[i]->get_dist(input);
                if(cur_dist < radius)
                {
                    neighbors.push_back(tree[i]);
                }
            }
            return neighbors;
        }

        arm_state* get_cheapest_parent(arm_state* input, std::vector<arm_state*> neighbors)
        {
            double lowest_cost = 100000000000;
            arm_state* cheapest_neighbor = nullptr;

            for(int i = 0; i < neighbors.size(); ++i)
            {
                double temp_cost = neighbors[i]->get_cost()+input->get_dist(neighbors[i]);
                if(temp_cost<lowest_cost)
                {
                    if(is_obstacle_free(input,neighbors[i]))
                    {
                        cheapest_neighbor = neighbors[i];
                        lowest_cost = temp_cost;
                    }
                    
                }
            }
            if(nullptr == cheapest_neighbor)
            {
                printf("FAILED TO FIND CHEAPEST PARENT!!\n");
            }
            return cheapest_neighbor;
        }

        bool is_parent(arm_state* child_candidate, arm_state* parent_candidate)
        {
            if(child_candidate->get_step_num() + 1 == parent_candidate->get_step_num() )
            {
                return true;
            }
            return false;
        }

        bool is_child(arm_state* parent_candidate, arm_state* child_candidate)
        {
            if(child_candidate->get_step_num() + 1 == parent_candidate->get_step_num() )
            {
                return true;
            }
            return false;
        }

        void rewire(arm_state* new_node, std::vector<arm_state*> regional_neighbors)
        {
            for(int i = 0; i < regional_neighbors.size(); i++)
            {
                if(regional_neighbors[i]->get_cost() > new_node->get_cost() + new_node->get_dist(regional_neighbors[i]) && is_obstacle_free(new_node,regional_neighbors[i])) // see if the cost would be imprroved by making the new node the parent of an existing node
                {
                    make_new_parent(new_node, regional_neighbors[i]);
                }
            }
        }

        void make_new_parent(arm_state* new_parent, arm_state* new_child)
        {
            //get the current parent of new child, remove that one, attach the child to new_parent and then update the children of new-child
            arm_state* current_parent = new_child->get_parent();

            new_child->remove_neighbor(current_parent);
            current_parent->remove_neighbor(new_child);

            new_parent->add_neighbor(new_child);
            new_child->add_neighbor(new_parent);

            update_children(new_child);
            printf("All children updated.");


        }

        arm_state* find_parent(arm_state* child)
        {
            for(int i = 0; i < child->get_num_neighbors(); ++i)
            {
                if(is_parent(child, child->get_neighbor(i)))
                {
                    return child->get_neighbor(i);
                }
            }
            printf("Failed to find parent.\n");
        }

        void update_children(arm_state* parent)
        {
            for(int i = 0; i < parent->get_num_neighbors(); i++)
            {
                if(is_child(parent,parent->get_neighbor(i)))
                {
                    arm_state* child = parent->get_neighbor(i);
                    child->set_step_num(parent->get_step_num()+1);
                    child->set_cost(parent->get_cost()+parent->get_dist(child));
                    update_children(child);
                }
            }
        }

        bool is_obstacle_free(arm_state* start, arm_state* end)
        {
            std::vector<double> unit_step = get_unit_step(start,end);
            
            std::vector<double> nearest_state = start->get_state(); 
            std::vector<double> last_valid = start->get_state();
            std::vector<double> temp_state = start->get_state();
            

            for(int i = 0; i < steps; ++i)
            {                
                for(int i = 0; i < temp_state.size(); ++i)
                {
                    temp_state[i] += unit_step[i];
                }

                if(IsValidArmConfiguration(temp_state))
                {
                    // std::cout << "\t\tThis is a valid config. \n" <<std::endl;
                    last_valid = temp_state;
                    // last_valid.assign(temp_state.begin(), temp_state.end()); //overwrites last_valid with temp_state
                    
                    if(states_are_equal(end,last_valid))
                    {
                        return true;
                    }
                }
                else
                {
                    // std::cout << "\t\tNOT VALID \n" <<std::endl;
                    return false;
                }
            }
            return false; 
        }

};


class prm:public rrt_star
{
    protected:
        int total_nodes = 10000;
        double alpha = PI/3; //radius in which a node can be considered a in the same "neighborhood"
        std::vector<arm_state*> node_list;
        int tree_count = 1; //in this, we use "tree" and "cluster" interchangably. Each node tracks its cluster affiliation in tree_id
        int live_trees = 0; //some trees will get absorbed into one another. this tracks the total remaining.
        int max_neighbors = 15;

    public:
        prm(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
                    int numofDOFs, double*** plan, int* planlength):
                    rrt_star(map, x_size, y_size,armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength)
                    {
                       //do nothing
                    }

        void build_roadmap()
        {
            for(int i = 0; i < total_nodes; ++i)
            {
                // printf("Iteration %d", i);
                arm_state* new_state = gen_rand_state_no_seed();

                while(!IsValidArmConfiguration(new_state))
                {
                    new_state = gen_rand_state_no_seed();
                }

                new_state->set_cost(1000000);
                std::vector<arm_state*> neighbors = neighbors_in_alpha(new_state);

                if(neighbors.empty()) //means that it is isolated and forms a new cluster.
                {
                    
                    new_state->set_tree_id(tree_count);
                    tree_count++;
                    live_trees++;
                    // printf("\tNew tree created (%d)\n", live_trees);
                }
                else //found other nodes to connect to
                {
                    // printf("\t%d neighbors found \n",neighbors.size());
                    for(int j = 0; j <neighbors.size(); j++)
                    {
                        // printf("%s %d\n",__FUNCTION__, __LINE__);
                        // if(!new_state->in_same_cluster(neighbors[j]) && is_obstacle_free(new_state,neighbors[j]))
                        if(!same_cluster_id(neighbors[j],new_state) && is_obstacle_free(new_state,neighbors[j]))
                        {
                            // printf("%s %d\n",__FUNCTION__, __LINE__);
                            connect_states(new_state,neighbors[j]);
                            // printf("%s %d\n",__FUNCTION__, __LINE__);
                        }
                        // printf("%s %d\n",__FUNCTION__, __LINE__);

                    }
                }
                node_list.push_back(new_state);
            }
            get_tree_info();
        }

        arm_state* find_nearest(arm_state* input)
        {
            int nearest_index = 0;
            double nearest_dist = 10000;

            for(int i = 0; i < node_list.size(); ++i)
            {
                double cur_dist = node_list[i]->get_dist(input);
                if(cur_dist < nearest_dist)
                {
                    nearest_dist = cur_dist;
                    nearest_index = i;
                }
            }
            return node_list[nearest_index];
        }

        void generate_plan()
        {
            arm_state* nearest_start = find_nearest(start_state);
            arm_state* nearest_goal = find_nearest(goal_state);

            if(nearest_goal->get_tree_id() != nearest_start->get_tree_id())
            {
                total_nodes = total_nodes/4;
                while(nearest_goal->get_tree_id() != nearest_start->get_tree_id())
                {
                    printf("Path not achievable with current tree. Adding more nodes.\n");
                    build_roadmap();
                    nearest_start = find_nearest(start_state);
                    nearest_goal = find_nearest(goal_state);
                }
                printf("Path is now achievable. \n");

            }
            else
            {
                printf("Path is achievable\n");
            }

            goal_state->add_neighbor(nearest_goal);
            nearest_goal->add_neighbor(goal_state);

            start_state->add_neighbor(nearest_start);
            nearest_start->add_neighbor(start_state);



        }

        void get_tree_info()
        {
            int max = -1;
            for(int i = 0; i < node_list.size(); ++i)
            {
                max = std::max(max,node_list[i]->get_tree_id());
            }
            printf("Max tree index = %d    Live tree count = %d\n",max, live_trees);
        }

        std::vector<arm_state*> neighbors_in_alpha(arm_state* input)
        {
            std::vector<arm_state*> neighbors; 
            int neighbor_ct = 0;
            for(int i = 0; i < node_list.size(); ++i)
            {
                if(node_list[i] != input)
                {
                    double cur_dist = node_list[i]->get_dist(input);
                    if(cur_dist < alpha)
                    {
                        // printf(" FOUND NEIGHBOR!\n");
                        neighbor_ct++;
                        neighbors.push_back(node_list[i]);
                    }
                    if(neighbor_ct >= max_neighbors)
                    {
                        return neighbors;
                    }
                }
               
            }
            return neighbors;
        }


        // void connect_states(arm_state* node1, arm_state* node2)
        // {
        //     if(-1 == node1->get_tree_id() ) //not yet part of an existing tree.
        //     {
        //         printf("%s %d\n",__FUNCTION__, __LINE__);   
        //         node1->add_cluster(node2->get_top_cluster());
        //         printf("%s %d\n",__FUNCTION__, __LINE__);
        //         node1->add_neighbor(node2);
        //         node2->add_neighbor(node1);
        //     }
        //     else //need to combine trees.
        //     {
        //         live_trees--;
        //         node1->add_cluster(node2->get_top_cluster());
        //         node1->add_neighbor(node2);
        //         node2->add_neighbor(node1);
        //         update_cluster_group(node1,node2->get_top_cluster());
        //         printf("\tTree absorbed (%d)\n", live_trees);
        //     }
        // }
        
        void connect_states(arm_state* node1, arm_state* node2)
        {
            if(-1 == node1->get_tree_id() ) //not yet part of an existing tree.
            {
                // printf("%s %d\n",__FUNCTION__, __LINE__);   
                node1->set_tree_id(node2->get_tree_id());
                // printf("%s %d\n",__FUNCTION__, __LINE__);
                node1->add_neighbor(node2);
                node2->add_neighbor(node1);
            }
            else //need to combine trees.
            {

                // node1->set_tree_id(node2->get_tree_id());
                node1->add_neighbor(node2);
                node2->add_neighbor(node1);

                update_cluster_group(node1,node2->get_tree_id());
                live_trees--;

                // printf("\tTree absorbed (%d)\n", live_trees);
            }
        }

        // void update_cluster_group(arm_state* input, int cluster_id) //recursively updates the clusters
        // {
        //      if(!input->in_cluster(cluster_id))
        //     {
        //         input->add_cluster(cluster_id);
        //         for(int i = 0; i < input->get_num_neighbors(); ++i)
        //         {
        //             update_cluster_group(input->get_neighbor(i), cluster_id);
        //         }
        //     }
        // }
        void update_cluster_group(arm_state* input, int cluster_id) //recursively updates the clusters
        {
             if(input->get_tree_id() != cluster_id)
            {
                input->set_tree_id(cluster_id);
                for(int i = 0; i < input->get_num_neighbors(); ++i)
                {
                    update_cluster_group(input->get_neighbor(i), cluster_id);
                }
            }
        }

        bool same_cluster_id(arm_state* node1, arm_state* node2)
        {
            return node1->get_tree_id() == node2->get_tree_id();
        }
        

    //Djikstra-related functions
        
        struct compare_costs
        {
            bool operator()(arm_state* const& left, arm_state* const& right)
            {
                if(left->get_cost() != right->get_cost())
                {
                    return left->get_cost() > right->get_cost();
                }
                else
                {
                    //tie-breaking if cost is the same
                    return false;
                }
            }     
        };
        
        std::priority_queue<arm_state*, std::vector<arm_state*>, compare_costs>>open_list;      
        std::vector<arm_state*> closed_list;

        
        void run_djikstra()
        {

        }

};