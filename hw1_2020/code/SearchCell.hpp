#include <math.h>

class SearchCell
{
private:
    /* data */
public:
    int x_pos, y_pos;
    double g_val;
    int cost_val;
    SearchCell* parent;
    int t_val;
    //SearchCell* initCell();
    ~SearchCell();
};

SearchCell::~SearchCell()
{
    delete (this->parent);
}