#include "SOP01.h"

SOP01::SOP01(OP_Network *parent, const char *name, OP_Operator *entry)
		: SOP_Node(parent, name, entry)
{}

SOP01::~SOP01() = default;

auto SOP01::cookMySop(OP_Context &context) -> OP_ERROR
{
	return error();
}
