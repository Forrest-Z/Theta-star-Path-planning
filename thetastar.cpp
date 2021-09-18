#include<global_planner/thetastar.h>
#include<costmap_2d/cost_values.h>

namespace global_planner {

thetaStarExpansion::thetaStarExpansion(PotentialCalculator* p_calc, int xs, int ys) :
        Expander(p_calc, xs, ys) {
}

bool thetaStarExpansion::calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));

    std::fill(potential, potential + ns_, POT_HIGH);
    potential[start_i] = 0;

    int goal_i = toIndex(end_x, end_y);
    int cycle = 0;
	i=start_i;
    while (queue_.size() > 0 && cycle < cycles) {
        Index top = queue_[0];
	std::pop_heap(queue_.begin(), queue_.end(), greater1());
	int new_i = top.i;
	double x1 = i % nx_, y1 = i / nx_;
	double x2 = new_i % nx_, y2 = new_i / nx_;
        if(LineOfSight(x1, y1, x2, y2, costs)){
        	     	
		if (new_i == goal_i)
            		return true;
            
        	add(costs, potential, potential[new_i], new_i + 1, end_x, end_y);
        	add(costs, potential, potential[new_i], new_i - 1, end_x, end_y);
        	add(costs, potential, potential[new_i], new_i + nx_, end_x, end_y);
        	add(costs, potential, potential[new_i], new_i - nx_, end_x, end_y);
		}else{
		
        	queue_.pop_back();
        	
        	i = top.i;
        	
		if (i == goal_i)
            		return true;

        	add(costs, potential, potential[i], i + 1, end_x, end_y);
        	add(costs, potential, potential[i], i - 1, end_x, end_y);
        	add(costs, potential, potential[i], i + nx_, end_x, end_y);
        	add(costs, potential, potential[i], i - nx_, end_x, end_y);
		}
		
        cycle++;
    }

    return false;
}

void thetaStarExpansion::add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x,
                         int end_y) {
    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH)
        return;

    if(costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
        return;

    potential[next_i] = p_calc_->calculatePotential(potential, costs[next_i] + neutral_cost_, next_i, prev_potential);
    int x = next_i % nx_, y = next_i / nx_;
    float distance = estimateValue(end_x, end_y, x, y);

    queue_.push_back(Index(next_i, potential[next_i] + distance * neutral_cost_));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

int thetaStarExpansion::estimateValue(int x1, int y1, int x2, int y2)
{
    int dx = abs(x1-x2);
    int dy = abs(y1-y2);
    if(dx > dy)
         return 10*dx+4*dy;
    else
         return 10*dy+4*dx;  
}

int AStarExpansion::LineOfSight(double x1, double y1, double x2, double y2, unsigned char* costs)
{
    int dy = y2 - y1;
    int dx = x2 - x1;

    int f = 0;
    int sy, sx; 

    int x_offset, y_offset;

    if (dy < 0) {
        dy = -dy;
        sy = -1;
        y_offset = 0; 
    }
    else {
        sy = 1;
        y_offset = 1; 
    }

    if (dx < 0) {
        dx = -dx;
        sx = -1;
        x_offset = 0; 
    }
    else {
        sx = 1;
        x_offset = 1; 
    }

    if (dx >= dy) { 
        while (x1 != x2) {
            f = f + dy;
            if (f >= dx) {
            	int next_i = toIndex(x1 + x_offset, y1 + y_offset);
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
                    return 0;

                y1 = y1 + sy;
                f = f - dx;
            }

            if (f != 0) {
				int next_i = toIndex(x1 + x_offset, y1 + y_offset);  
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
                    return 0;
            }

            if (dy == 0) {
				int next_i = toIndex(x1 + x_offset, y1);
				int next_j = toIndex(x1 + x_offset, y1 + 1);
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION) && 
				costs[next_j]>=lethal_cost_ && !(unknown_ && costs[next_j]==costmap_2d::NO_INFORMATION))
                    return 0;
            }

            x1 += sx;
        }
    }

    else {
        while (y1 != y2) {
            f = f + dx;
            if (f >= dy) {
            	int next_i = toIndex(x1 + x_offset, y1 + y_offset);  
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
                    return 0;

                x1 = x1 + sx;
                f = f - dy;
            }

            if (f != 0) {
            	int next_i = toIndex(x1 + x_offset, y1 + y_offset);  
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION))
                    return 0;
            }

            if (dx == 0) {
            	int next_i = toIndex(x1, y1 + y_offset);
				int next_j = toIndex(x1 + 1, y1 + y_offset);
                if (costs[next_i]>=lethal_cost_ && !(unknown_ && costs[next_i]==costmap_2d::NO_INFORMATION) && 
				costs[next_j]>=lethal_cost_ && !(unknown_ && costs[next_j]==costmap_2d::NO_INFORMATION))
                    return 0;
            }

            y1 += sy;
        }
    }
    return 1;
}

} //end namespace global_planner
