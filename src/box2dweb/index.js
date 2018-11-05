/*
* Copyright (c) 2006-2007 Erin Catto http://www.gphysics.com
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

import a from './a';
import b from './b';
import c from './c';
import d from './d';
import e from './e';
import f from './f';
import g from './g';
import h from './h';
import i from './i';
import j from './j';
import k from './k';

export const Box2D = {};

a(Box2D);

//#TODO remove assignments from global namespace
var Vector = Array;
var Vector_a2j_Number = Box2D.NVector;
//package structure
if (typeof Box2D === 'undefined') Box2D = {};
if (typeof Box2D.Collision === 'undefined') Box2D.Collision = {};
if (typeof Box2D.Collision.Shapes === 'undefined') Box2D.Collision.Shapes = {};
if (typeof Box2D.Common === 'undefined') Box2D.Common = {};
if (typeof Box2D.Common.Math === 'undefined') Box2D.Common.Math = {};
if (typeof Box2D.Dynamics === 'undefined') Box2D.Dynamics = {};
if (typeof Box2D.Dynamics.Contacts === 'undefined')
  Box2D.Dynamics.Contacts = {};
if (typeof Box2D.Dynamics.Controllers === 'undefined')
  Box2D.Dynamics.Controllers = {};
if (typeof Box2D.Dynamics.Joints === 'undefined') Box2D.Dynamics.Joints = {};

//pre-definitions
b(Box2D);

//definitions
Box2D.postDefs = [];

c(Box2D);
d(Box2D);
e(Box2D);
f(Box2D);
g(Box2D);
h(Box2D);
i(Box2D);
j(Box2D);
k(Box2D);

//post-definitions
for (let i = 0; i < Box2D.postDefs.length; ++i) Box2D.postDefs[i]();
delete Box2D.postDefs;

console.log(Box2D);
