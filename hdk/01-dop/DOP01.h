#ifndef INC_01_DOP_DOP01_H
#define INC_01_DOP_DOP01_H

#include <DOP/DOP_Node.h>

class DOP01 : public DOP_Node
{
public:
	DOP01(OP_Network *net, const char *name, OP_Operator *entry);
};

#endif //INC_01_DOP_DOP01_H
