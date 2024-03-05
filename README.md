# HinaPE is Not a Physics Engine

Designed for Houdini HDK version

## Introduction

HinaPE is a recursive acronym for "HinaPE is not a Physics Engine". 

## Build Instruction

Dependencies:

- Houdini (v20.6.25)
- tbb (use Houdini bundled tbb)
- CUDA (>=12.1)

```cpp
cmake -B Build -S .
cmake --build Build
```

## TODO List

Fluid

- [x] DFSPH

- [ ] PBF

- [ ] WCSPH

- [ ] IISPH

- [ ] Smoke / Fire

- [ ] FLIP

- [ ] APIC

Boundary

- [x] Akinci 2012

- [ ] Bender 2019

- [ ] Chang 2020

- [ ] Ruan 2021

Emitter

- [ ] Implicit Surface (Select Surface Type)

- [ ] Explicit Surface (Load SOP)

Two-way Coupling

- [x] Rigidbody proxy
