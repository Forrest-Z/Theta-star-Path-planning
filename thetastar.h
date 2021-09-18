#ifndef _THETASTAR_H
#define _THETASTAR_H

#include <global_planner/planner_core.h>
#include <global_planner/expander.h>
#include <vector>
#include <algorithm>

namespace global_planner {
class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
        bool operator()(const Index& a, const Index& b) const {
            return a.cost > b.cost;
        }
};

class AStarExpansion : public Expander {
    public:
        AStarExpansion(PotentialCalculator* p_calc, int nx, int ny);
        bool calculatePotentials(unsigned char* costs, double start_x, double start_y, double end_x, double end_y, int cycles,
                                float* potential);
        int estimateValue(int x1, int y1, int x2, int y2);                        
    	int LineOfSight(double x1, double y1, double x2, double y2,unsigned char* costs);
    private:
        void add(unsigned char* costs, float* potential, float prev_potential, int next_i, int end_x, int end_y);
		int i; 
        std::vector<Index> queue_;
};

} //end namespace global_planner
#endif

