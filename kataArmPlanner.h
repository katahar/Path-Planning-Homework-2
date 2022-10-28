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
#include <algorithm>    // std::min
#include <assert.h>     /* assert */


class arm_state
{
    protected:
        int DOF;
        int step_num = 0; 
        std::vector<double> state;
        bool is_goal = false;
        
        int num_neighbors = 0;
        std::vector<arm_state*> connected_neighbors;

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
                // std::srand(time(NULL));
                std::srand(1000);
            }
        
        arm_state* gen_rand_state()
        {
            if(std::rand()%10 == 1)
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


class rrt:base_planner
{
    protected:
        double epsillon = 1.2;
        int steps = 10; //number of intermediate steps along epsillon that are checked for collision
        std::vector<arm_state*> tree;
        bool goal_found = false;
        double min_dist = 1000000;
        std::vector<arm_state*> plan;
        int max_iterations = 100000;

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

class rrt_connect:base_planner
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
                if(TRAPPED != extend_tree(gen_rand_state(),tree_id, false)) //if not trapped after extending
                {
                    // printf("\trandom extension not trapped. attempting to connect to other tree.\n");
                    // print_trees();

                    if(REACHED == connect(get_last_extended(tree_id),switch_tree_id(tree_id)) )//keep extending until the other tree is reached
                    {

                        // print_trees();
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

        int connect(arm_state* last_extended, int tree_id) //you want to connect to the other tree, so last_extended lives on the opposite tree
        {
            // printf("\tCONNECT FUNCTION: INPUT TREE %d\n",tree_id);
            int cur_status = ADVANCED;
            arm_state* target = find_nearest(last_extended,tree_id);
            // int counter = 0;

            while(ADVANCED == cur_status)
            {
                printf("Extending tree %d to %d\n", switch_tree_id(tree_id), tree_id);
                // printf("\titeration %d\n",counter);
                // counter++;
                cur_status = extend_tree(last_extended,tree_id, true);
            }
            return cur_status;
        }

        int extend_tree(arm_state* rand_input, int tree_id, bool called_from_connect) 
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


                printf("neighbor from tree %d\n", tree_id);
                arm_state* new_state = new arm_state(last_valid);
                printf("\tNearest neighbor: ");
                nearest->print();

                printf("\tLast valid state: ");
                new_state->print();

                printf("\ttarget state: ");
                rand_input->print();          


                min_dist = std::min(min_dist,goal_state->get_dist(new_state));


                // new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                // nearest->add_neighbor(new_state);
    
                if(called_from_connect)
                {

                    arm_state* last_added_same_tree = get_last_extended((tree_id)); 
                    new_state->add_neighbor(last_added_same_tree); //adding each other as mutual neighbors
                    last_added_same_tree->add_neighbor(new_state);
                    new_state->set_step_num(last_added_same_tree->get_step_num()+1);
                    
                    if(new_state->get_dist(last_added_same_tree) > epsillon)
                    {
                        
                        printf("\t(original tree is %d\n", switch_tree_id(tree_id));
                        printf("\tAttaching to : ");
                        last_added_same_tree->print();
                        printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(last_added_same_tree) );
                        printf("tree: \n");
                        print_tree(switch_tree_id(tree_id));

                        printf("OTHER tree\n");
                        print_tree(tree_id);
                        printf("================\n");
                    }
                   

                }
                else
                {
                    new_state->add_neighbor(nearest); //adding each other as mutual neighbors
                    nearest->add_neighbor(new_state);
                    new_state->set_step_num(nearest->get_step_num()+1);

                    if(new_state->get_dist(nearest) > epsillon)
                    {       
                        printf("\t(original tree is %d\n", (tree_id));
                        printf("\tAttaching to : ");
                        nearest->print();
                        printf("\tepsillon %f, distance between neighbors: %f\n", epsillon, new_state->get_dist(nearest) );
                    }

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
                    goal_tree[i]->print();
                }
            }
            else
            {
                printf("printing start tree:(%d) \n", START_TREE);
                for(int i = 0; i <start_tree.size(); ++i)
                {
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
                printf("\t");
                plan[i]->print();
            }
        }
};