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
                std::srand(time(NULL));

            }

        arm_state* gen_rand_state()
        {
            std::vector<double> rand_angles;
            for(int i = 0; i < DOFs; ++i)
            {
                rand_angles.push_back(2*3.14159*(std::rand()%1000)/1000); //generates a number in [0,2pi), up to 3 decimal places
            }
            arm_state* generated_state = new arm_state(rand_angles);
            return generated_state;
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
        
        
        #define PI 3.141592654
        //the length of each link in the arm
        #define LINKLENGTH_CELLS 10
        #define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

        int IsValidArmConfiguration(arm_state* state)
        {
            IsValidArmConfiguration(state, DOFs,map,x_size,y_size);
        }

        int IsValidArmConfiguration(arm_state* state, int numofDOFs, double*	map,
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
       
        int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
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

};


class rrt:base_planner
{
    protected:
        double epsillon;
        std::vector<arm_state*> tree;

    public:
        
        rrt(double* map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad,
            int numofDOFs, double*** plan, int* planlength):
            base_planner(map, x_size, y_size,armstart_anglesV_rad, armgoal_anglesV_rad, numofDOFs, plan, planlength)
            {
                tree.push_back(this->start_state);
            }

        void build_rrt()
        {
            for(int i = 0; i < 100; i++)
            {
                extend_tree(gen_rand_state());
            }
        } 

        void extend_tree(arm_state* input)
        {
            arm_state* nearest = find_nearest(input);
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
};

class arm_state
{
    protected:
        int DOF; 
        std::vector<double> state;
        
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
                state.push_back(anglesV_rad[i]);
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

        std::vector<double> get_state()
        {
        return state;
        }

        double get_angle(int index)
        {
            if(index<DOF)
            {
                return state[index];
            }
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
            return -2.0;
        }
        arm_state* get_ptr()
        {
            return this;
        }
};