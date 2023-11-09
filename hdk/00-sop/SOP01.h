#ifndef INC_00_SOP_SOP01_H
#define INC_00_SOP_SOP01_H

#include <SOP/SOP_Node.h>

class SOP01 : public SOP_Node
{
public:
	SOP01(OP_Network *parent, const char *name, OP_Operator *entry);
	~SOP01() override;

protected:
	auto cookMySop(OP_Context &context) -> OP_ERROR override;
};

#endif //INC_00_SOP_SOP01_H
