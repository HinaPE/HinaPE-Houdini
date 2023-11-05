#ifndef HINAPE_HOUDINI_ATTRIBUTE_HELPER_H
#define HINAPE_HOUDINI_ATTRIBUTE_HELPER_H

#include <SIM/SIM_GeometryCopy.h>
#include <GU/GU_Detail.h>

template <typename HandleType>
HandleType PointAttribute(GU_Detail& gdp, const char* name)
{
	auto attr = gdp.findAttribute(GA_ATTRIB_POINT, name);
	return HandleType(attr);
}

template <typename HandleType>
HandleType VertexAttribute(const GU_Detail& gdp, const char* name)
{
	auto attr = gdp.findAttribute(GA_ATTRIB_VERTEX, name);
	return HandleType(attr);
}

template <typename HandleType>
HandleType PrimitiveAttribute(const GU_Detail& gdp, const char* name)
{
	auto attr = gdp.findAttribute(GA_ATTRIB_PRIMITIVE, name);
	return HandleType(attr);
}

template <typename HandleType>
HandleType DetailAttribute(const GU_Detail& gdp, const char* name)
{
	auto attr = gdp.findAttribute(GA_ATTRIB_DETAIL, name);
	return HandleType(attr);
}

#endif //HINAPE_HOUDINI_ATTRIBUTE_HELPER_H
