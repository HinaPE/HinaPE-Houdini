# Chapter 1: Houdini Node System

## Attributes

### Attribute Wrangle

> This is a very powerful, low-level node that lets experts who are familiar with **<u>VEX tweak attributes</u>** using code.

- Use `MMB` to view compiled result

### VEX Expression

> VEX is a high-performance expression language used in many places in Houdini, such as writing shaders.

#### Overview

Several nodes in Houdini let you write short VEX expressions or snippets of VEX code. For example,

- Attrib Wrangle geometry nodes

- Geometry Wrangle

- Gas Field Wrangle dynamics nodes

- particle dynamics nodes

These VEX expressions run on each element (point, particle, primitive, voxel, depending on the node type) passing through the node. The code can read the values of node parameters and geometry attributes, and set special variables to change values in the input geometry.

- PS: The Python SOP is similar but lets you edit geometry using a Python snippet.

Modeling – The VEX SOP(**deprecated**) allows you to write a custom surface node that manipulates point attributes. This can <u>move points around</u>, <u>adjust velocities</u>, <u>change colors</u>. As well, you can <u>group points</u> or do many other useful tasks. VEX SOPs typically run 10 or more times faster than a point SOP.

#### Features

- Ad-hoc

- Automatically supports threading and parallel computation

- Working directly on attributes instead of local variables

- Simpler than the equivalent HScript expressions

- VEX operations are supported inside compiled SOP blocks, but HScript expressions using local variables cannot be compiled

#### Grammar

1. Each statement **<u>must</u>** end with a semicolon (`;`)

2. Attribute Access: 
   
   - `@P.x`
   
   - `@P={0, 1, 0}`

3. Comments: `//` and `/* ... */`

4. `sin` & `cos` use radians, not degrees

VEX > Using VEX expressions

## Point Cloud
