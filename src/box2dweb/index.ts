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

// tslint:disable-next-line no-empty
const Ctor = (function inheritor() {} as any) as { new (): any };

// package structure
export const Box2D = {
  Collision: {
    b2AABB: null,
    Shapes: { b2PolygonShape: null, b2CircleShape: null },
  },

  Common: {
    Math: {
      b2Vec2: null,
    },
  },

  Dynamics: {
    b2BodyDef: null,
    b2Body: null,
    b2FixtureDef: null,
    b2World: null,
    b2DebugDraw: null,

    Contacts: null,
    Controllers: null,
    Joints: { b2MouseJointDef: null },
  },

  postDefs: [],

  inherit(ctor: () => void, base: () => void) {
    const tempCtor = ctor;
    Ctor.prototype = base.prototype;
    ctor.prototype = new Ctor();
    ctor.prototype.constructor = tempCtor;
  },

  generateCallback(context: any, cb: () => void) {
    return () => {
      cb.apply(context, arguments);
    };
  },

  NVector(length = 0) {
    return new Array(length).fill(0);
  },

  is(o1: any, o2: any) {
    if (o1 === null) {
      return false;
    }

    if (o2 instanceof Function && o1 instanceof o2) {
      return true;
    }

    if (
      o1.constructor.__implements !== undefined &&
      o1.constructor.__implements[o2]
    ) {
      return true;
    }

    return false;
  },

  parseUInt(v: number) {
    return Math.abs(parseInt(`${v}`, 10));
  },
};

a(Box2D);

// pre-definitions
b(Box2D);

// definitions
c(Box2D);
d(Box2D);
e(Box2D);
f(Box2D);
g(Box2D);
h(Box2D);
i(Box2D);
j(Box2D);
k(Box2D);

// post-definitions
for (const postDef of Box2D.postDefs as Array<() => void>) {
  postDef();
}
delete Box2D.postDefs;

console.log(Box2D); // tslint:disable-line no-console
