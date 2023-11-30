# Macros

## GA_GBMacros.h

### `GA_FOR_ALL_PTOFF(GU_Detail*, GA_Offset)`

> Iterate over all points via offsets.

```cpp
const GU_Detail *gdp = geo.getGdp();
GA_Offset offset;
GA_FOR_ALL_PTOFF(gdp, offset)
{
    UT_Vector4 pos = gdp->getPos4(offset);
}
```


































