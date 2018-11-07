import Color from './common/color';
import * as Maths from './common/math';
import Mat22 from './common/math/mat22';
import Mat33 from './common/math/mat33';
import Sweep from './common/math/sweep';
import Transform from './common/math/transform';
import Vec2 from './common/math/vec2';
import Vec3 from './common/math/vec3';
import * as Settings from './common/settings';
import DebugDraw from './dynamics/debug-draw';
import TimeStep from './dynamics/time-step';

import Aabb from './collision/aabb';
import Bound from './collision/bound';
import BoundValues from './collision/bound-values';
import ClipVertex from './collision/clip-vertex';
import ContactId from './collision/contact-id';
import ContactPoint from './collision/contact-point';
import Features from './collision/features';
import * as Collision from './collision';
import ManifoldPoint from './collision/manifold-point';
import Manifold from './collision/manifold';
import SimplexCache from './collision/simplex-cache';
import SimplexVertex from './collision/simplex-vertex';

/**
 * @see: ./c.js
 */
import * as Distance from './collision/distance';
import DistanceInput from './collision/distance-input';
import DistanceOutput from './collision/distance-output';
import DistanceProxy from './collision/distance-proxy';
// import DynamicTree from './collision/dynamic-tree';
// import DynamicTreeBroadPhase from './collision/dynamic-tree-broad-phase';
// import DynamicTreeNode from './collision/dynamic-tree-node';
// import DynamicTreePair from './collision/dynamic-tree-pair';
import Point from './collision/point';
import RayCastInput from './collision/ray-cast-input';
import RayCastOutput from './collision/ray-cast-output';
// import Segment from './collision/segment';
// import SeparationFunction from './collision/separation-function';
import Simplex from './collision/simplex';
// import TimeOfImpact from './collision/time-of-impact';
// import ToiInput from './collision/toi-input';
// import WorldManifold from './collision/world-manifold';

/**
 * @see: ./d.js
 */
import CircleShape from './collision/shapes/circle-shape';
import EdgeChainDef from './collision/shapes/edge-chain-def';
import EdgeShape from './collision/shapes/edge-shape';
import MassData from './collision/shapes/mass-data';
import PolygonShape from './collision/shapes/polygon-shape';
import Shape from './collision/shapes/shape';

/**
 * @see: ./g.js
 */
// import Body from './dynamics/body';
// import BodyDef from './dynamics/body-def';
// import ContactFilter from './dynamics/contact-filter';
// import ContactImpulse from './dynamics/contact-impulse';
// import ContactListener from './dynamics/contact-listener';
// import ContactManager from './dynamics/contact-manager';
// import DestructionListener from './dynamics/destruction-listener';
// import FilterData from './dynamics/filter-data';
// import Fixture from './dynamics/fixture';
// import FixtureDef from './dynamics/fixture-def';
// import Island from './dynamics/island';
// import World from './dynamics/world';

/**
 * @see: ./h.js
 */
// import CircleContact from './dynamics/contacts/circle-contact';
// import Contact from './dynamics/contacts/contact';
// import ContactConstraint from './dynamics/contacts/contact-constraint';
// import ContactConstraintPoint from './dynamics/contacts/contact-constraint-point';
// import ContactEdge from './dynamics/contacts/contact-edge';
// import ContactFactory from './dynamics/contacts/contact-factory';
// import ContactRegister from './dynamics/contacts/contact-register';
// import ContactResult from './dynamics/contacts/contact-result';
// import ContactSolver from './dynamics/contacts/contact-solver';
// import EdgeAndCircleContact from './dynamics/contacts/edge-and-circle-contact';
// import NullContact from './dynamics/contacts/null-contact';
// import PolyAndCircleContact from './dynamics/contacts/poly-and-circle-contact';
// import PolyAndEdgeContact from './dynamics/contacts/poly-and-edge-contact';
// import PolygonContact from './dynamics/contacts/polygon-contact';
// import PositionSolverManifold from './dynamics/contacts/position-solver-manifold';

/**
 * @see: ./i.js
 */
// import BuoyancyController from './dynamics/controllers/buoyancy-controller';
// import ConstantAccelController from './dynamics/controllers/constant-accel-controller';
// import ConstantForceController from './dynamics/controllers/constant-force-controller';
// import Controller from './dynamics/controllers/controller';
// import ControllerEdge from './dynamics/controllers/controller-edge';
// import GravityController from './dynamics/controllers/gravity-controller';
// import TensorDampingController from './dynamics/controllers/tensor-damping-controller';

/**
 * @see: ./j.js
 */
// import DistanceJoint from './dynamics/controllers/distance-joint';
// import DistanceJointDef from './dynamics/controllers/distance-joint-def';
// import FrictionJoint from './dynamics/controllers/friction-joint';
// import FrictionJointDef from './dynamics/controllers/friction-joint-def';
// import GearJoint from './dynamics/controllers/gear-joint';
// import GearJointDef from './dynamics/controllers/gear-joint-def';
// import Jacobian from './dynamics/controllers/jacobian';
// import Joint from './dynamics/controllers/joint';
// import JointDef from './dynamics/controllers/joint-def';
// import JointEdge from './dynamics/controllers/joint-edge';
// import LineJoint from './dynamics/controllers/line-joint';
// import LineJointDef from './dynamics/controllers/line-joint-def';
// import MouseJoint from './dynamics/controllers/mouse-joint';
// import MouseJointDef from './dynamics/controllers/mouse-joint-def';
// import PrismaticJoint from './dynamics/controllers/prismatic-joint';
// import PrismaticJointDef from './dynamics/controllers/prismatic-joint-def';
// import PulleyJoint from './dynamics/controllers/pulley-joint';
// import PulleyJointDef from './dynamics/controllers/pulley-joint-def';
// import RevoluteJoint from './dynamics/controllers/revolute-joint';
// import RevoluteJointDef from './dynamics/controllers/revolute-joint-def';
// import WeldJoint from './dynamics/controllers/weld-joint';
// import WeldJointDef from './dynamics/controllers/weld-joint-def';

export default function b(Box2D) {
  Box2D.Collision.IBroadPhase = 'Box2D.Collision.IBroadPhase';
  Box2D.Collision.b2AABB = Aabb;
  Box2D.Collision.b2Bound = Bound;
  Box2D.Collision.b2BoundValues = BoundValues;
  Box2D.Collision.b2Collision = Collision;
  Box2D.Collision.b2ContactID = ContactId;
  Box2D.Collision.b2ContactPoint = ContactPoint;
  Box2D.Collision.b2Distance = Distance;
  Box2D.Collision.b2DistanceInput = DistanceInput;
  Box2D.Collision.b2DistanceOutput = DistanceOutput;
  Box2D.Collision.b2DistanceProxy = DistanceProxy;

  function b2DynamicTree() {
    b2DynamicTree.b2DynamicTree.apply(this, arguments);
    if (this.constructor === b2DynamicTree)
      this.b2DynamicTree.apply(this, arguments);
  }
  Box2D.Collision.b2DynamicTree = b2DynamicTree;

  function b2DynamicTreeBroadPhase() {
    b2DynamicTreeBroadPhase.b2DynamicTreeBroadPhase.apply(this, arguments);
  }
  Box2D.Collision.b2DynamicTreeBroadPhase = b2DynamicTreeBroadPhase;

  function b2DynamicTreeNode() {
    b2DynamicTreeNode.b2DynamicTreeNode.apply(this, arguments);
  }
  Box2D.Collision.b2DynamicTreeNode = b2DynamicTreeNode;

  function b2DynamicTreePair() {
    b2DynamicTreePair.b2DynamicTreePair.apply(this, arguments);
  }
  Box2D.Collision.b2DynamicTreePair = b2DynamicTreePair;

  Box2D.Collision.b2Manifold = Manifold;
  Box2D.Collision.b2ManifoldPoint = ManifoldPoint;
  Box2D.Collision.b2Point = Point;
  Box2D.Collision.b2RayCastInput = RayCastInput;
  Box2D.Collision.b2RayCastOutput = RayCastOutput;

  function b2Segment() {
    b2Segment.b2Segment.apply(this, arguments);
  }
  Box2D.Collision.b2Segment = b2Segment;

  function b2SeparationFunction() {
    b2SeparationFunction.b2SeparationFunction.apply(this, arguments);
  }
  Box2D.Collision.b2SeparationFunction = b2SeparationFunction;

  Box2D.Collision.b2Simplex = Simplex;
  Box2D.Collision.b2SimplexCache = SimplexCache;
  Box2D.Collision.b2SimplexVertex = SimplexVertex;

  function b2TimeOfImpact() {
    b2TimeOfImpact.b2TimeOfImpact.apply(this, arguments);
  }
  Box2D.Collision.b2TimeOfImpact = b2TimeOfImpact;

  function b2TOIInput() {
    b2TOIInput.b2TOIInput.apply(this, arguments);
  }
  Box2D.Collision.b2TOIInput = b2TOIInput;

  function b2WorldManifold() {
    b2WorldManifold.b2WorldManifold.apply(this, arguments);
    if (this.constructor === b2WorldManifold)
      this.b2WorldManifold.apply(this, arguments);
  }
  Box2D.Collision.b2WorldManifold = b2WorldManifold;

  Box2D.Collision.ClipVertex = ClipVertex;
  Box2D.Collision.Features = Features;
  Box2D.Collision.Shapes.b2CircleShape = CircleShape;
  Box2D.Collision.Shapes.b2EdgeChainDef = EdgeChainDef;
  Box2D.Collision.Shapes.b2EdgeShape = EdgeShape;
  Box2D.Collision.Shapes.b2MassData = MassData;
  Box2D.Collision.Shapes.b2PolygonShape = PolygonShape;
  Box2D.Collision.Shapes.b2Shape = Shape;
  Box2D.Common.b2internal = 'Box2D.Common.b2internal';
  Box2D.Common.b2Color = Color;
  Box2D.Common.b2Settings = Settings;
  Box2D.Common.Math.b2Mat22 = Mat22;
  Box2D.Common.Math.b2Mat33 = Mat33;
  Box2D.Common.Math.b2Math = Maths;
  Box2D.Common.Math.b2Sweep = Sweep;
  Box2D.Common.Math.b2Transform = Transform;
  Box2D.Common.Math.b2Vec2 = Vec2;
  Box2D.Common.Math.b2Vec3 = Vec3;

  function b2Body() {
    b2Body.b2Body.apply(this, arguments);
    if (this.constructor === b2Body) this.b2Body.apply(this, arguments);
  }
  Box2D.Dynamics.b2Body = b2Body;

  function b2BodyDef() {
    b2BodyDef.b2BodyDef.apply(this, arguments);
    if (this.constructor === b2BodyDef) this.b2BodyDef.apply(this, arguments);
  }
  Box2D.Dynamics.b2BodyDef = b2BodyDef;

  function b2ContactFilter() {
    b2ContactFilter.b2ContactFilter.apply(this, arguments);
  }
  Box2D.Dynamics.b2ContactFilter = b2ContactFilter;

  function b2ContactImpulse() {
    b2ContactImpulse.b2ContactImpulse.apply(this, arguments);
  }
  Box2D.Dynamics.b2ContactImpulse = b2ContactImpulse;

  function b2ContactListener() {
    b2ContactListener.b2ContactListener.apply(this, arguments);
  }
  Box2D.Dynamics.b2ContactListener = b2ContactListener;

  function b2ContactManager() {
    b2ContactManager.b2ContactManager.apply(this, arguments);
    if (this.constructor === b2ContactManager)
      this.b2ContactManager.apply(this, arguments);
  }
  Box2D.Dynamics.b2ContactManager = b2ContactManager;

  Box2D.Dynamics.b2DebugDraw = DebugDraw;

  function b2DestructionListener() {
    b2DestructionListener.b2DestructionListener.apply(this, arguments);
  }
  Box2D.Dynamics.b2DestructionListener = b2DestructionListener;

  function b2FilterData() {
    b2FilterData.b2FilterData.apply(this, arguments);
  }
  Box2D.Dynamics.b2FilterData = b2FilterData;

  function b2Fixture() {
    b2Fixture.b2Fixture.apply(this, arguments);
    if (this.constructor === b2Fixture) this.b2Fixture.apply(this, arguments);
  }
  Box2D.Dynamics.b2Fixture = b2Fixture;

  function b2FixtureDef() {
    b2FixtureDef.b2FixtureDef.apply(this, arguments);
    if (this.constructor === b2FixtureDef)
      this.b2FixtureDef.apply(this, arguments);
  }
  Box2D.Dynamics.b2FixtureDef = b2FixtureDef;

  function b2Island() {
    b2Island.b2Island.apply(this, arguments);
    if (this.constructor === b2Island) this.b2Island.apply(this, arguments);
  }
  Box2D.Dynamics.b2Island = b2Island;

  Box2D.Dynamics.b2TimeStep = TimeStep;

  function b2World() {
    b2World.b2World.apply(this, arguments);
    if (this.constructor === b2World) this.b2World.apply(this, arguments);
  }
  Box2D.Dynamics.b2World = b2World;

  function b2CircleContact() {
    b2CircleContact.b2CircleContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2CircleContact = b2CircleContact;

  function b2Contact() {
    b2Contact.b2Contact.apply(this, arguments);
    if (this.constructor === b2Contact) this.b2Contact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2Contact = b2Contact;

  function b2ContactConstraint() {
    b2ContactConstraint.b2ContactConstraint.apply(this, arguments);
    if (this.constructor === b2ContactConstraint)
      this.b2ContactConstraint.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactConstraint = b2ContactConstraint;

  function b2ContactConstraintPoint() {
    b2ContactConstraintPoint.b2ContactConstraintPoint.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactConstraintPoint = b2ContactConstraintPoint;

  function b2ContactEdge() {
    b2ContactEdge.b2ContactEdge.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactEdge = b2ContactEdge;

  function b2ContactFactory() {
    b2ContactFactory.b2ContactFactory.apply(this, arguments);
    if (this.constructor === b2ContactFactory)
      this.b2ContactFactory.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactFactory = b2ContactFactory;

  function b2ContactRegister() {
    b2ContactRegister.b2ContactRegister.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactRegister = b2ContactRegister;

  function b2ContactResult() {
    b2ContactResult.b2ContactResult.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactResult = b2ContactResult;

  function b2ContactSolver() {
    b2ContactSolver.b2ContactSolver.apply(this, arguments);
    if (this.constructor === b2ContactSolver)
      this.b2ContactSolver.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2ContactSolver = b2ContactSolver;

  function b2EdgeAndCircleContact() {
    b2EdgeAndCircleContact.b2EdgeAndCircleContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = b2EdgeAndCircleContact;

  function b2NullContact() {
    b2NullContact.b2NullContact.apply(this, arguments);
    if (this.constructor === b2NullContact)
      this.b2NullContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2NullContact = b2NullContact;

  function b2PolyAndCircleContact() {
    b2PolyAndCircleContact.b2PolyAndCircleContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2PolyAndCircleContact = b2PolyAndCircleContact;

  function b2PolyAndEdgeContact() {
    b2PolyAndEdgeContact.b2PolyAndEdgeContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = b2PolyAndEdgeContact;

  function b2PolygonContact() {
    b2PolygonContact.b2PolygonContact.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2PolygonContact = b2PolygonContact;

  function b2PositionSolverManifold() {
    b2PositionSolverManifold.b2PositionSolverManifold.apply(this, arguments);
    if (this.constructor === b2PositionSolverManifold)
      this.b2PositionSolverManifold.apply(this, arguments);
  }
  Box2D.Dynamics.Contacts.b2PositionSolverManifold = b2PositionSolverManifold;

  function b2BuoyancyController() {
    b2BuoyancyController.b2BuoyancyController.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2BuoyancyController = b2BuoyancyController;

  function b2ConstantAccelController() {
    b2ConstantAccelController.b2ConstantAccelController.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2ConstantAccelController = b2ConstantAccelController;

  function b2ConstantForceController() {
    b2ConstantForceController.b2ConstantForceController.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2ConstantForceController = b2ConstantForceController;

  function b2Controller() {
    b2Controller.b2Controller.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2Controller = b2Controller;

  function b2ControllerEdge() {
    b2ControllerEdge.b2ControllerEdge.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2ControllerEdge = b2ControllerEdge;

  function b2GravityController() {
    b2GravityController.b2GravityController.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2GravityController = b2GravityController;

  function b2TensorDampingController() {
    b2TensorDampingController.b2TensorDampingController.apply(this, arguments);
  }
  Box2D.Dynamics.Controllers.b2TensorDampingController = b2TensorDampingController;

  function b2DistanceJoint() {
    b2DistanceJoint.b2DistanceJoint.apply(this, arguments);
    if (this.constructor === b2DistanceJoint)
      this.b2DistanceJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2DistanceJoint = b2DistanceJoint;

  function b2DistanceJointDef() {
    b2DistanceJointDef.b2DistanceJointDef.apply(this, arguments);
    if (this.constructor === b2DistanceJointDef)
      this.b2DistanceJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2DistanceJointDef = b2DistanceJointDef;

  function b2FrictionJoint() {
    b2FrictionJoint.b2FrictionJoint.apply(this, arguments);
    if (this.constructor === b2FrictionJoint)
      this.b2FrictionJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2FrictionJoint = b2FrictionJoint;

  function b2FrictionJointDef() {
    b2FrictionJointDef.b2FrictionJointDef.apply(this, arguments);
    if (this.constructor === b2FrictionJointDef)
      this.b2FrictionJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2FrictionJointDef = b2FrictionJointDef;

  function b2GearJoint() {
    b2GearJoint.b2GearJoint.apply(this, arguments);
    if (this.constructor === b2GearJoint)
      this.b2GearJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2GearJoint = b2GearJoint;

  function b2GearJointDef() {
    b2GearJointDef.b2GearJointDef.apply(this, arguments);
    if (this.constructor === b2GearJointDef)
      this.b2GearJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2GearJointDef = b2GearJointDef;

  function b2Jacobian() {
    b2Jacobian.b2Jacobian.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2Jacobian = b2Jacobian;

  function b2Joint() {
    b2Joint.b2Joint.apply(this, arguments);
    if (this.constructor === b2Joint) this.b2Joint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2Joint = b2Joint;

  function b2JointDef() {
    b2JointDef.b2JointDef.apply(this, arguments);
    if (this.constructor === b2JointDef) this.b2JointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2JointDef = b2JointDef;

  function b2JointEdge() {
    b2JointEdge.b2JointEdge.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2JointEdge = b2JointEdge;

  function b2LineJoint() {
    b2LineJoint.b2LineJoint.apply(this, arguments);
    if (this.constructor === b2LineJoint)
      this.b2LineJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2LineJoint = b2LineJoint;

  function b2LineJointDef() {
    b2LineJointDef.b2LineJointDef.apply(this, arguments);
    if (this.constructor === b2LineJointDef)
      this.b2LineJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2LineJointDef = b2LineJointDef;

  function b2MouseJoint() {
    b2MouseJoint.b2MouseJoint.apply(this, arguments);
    if (this.constructor === b2MouseJoint)
      this.b2MouseJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2MouseJoint = b2MouseJoint;

  function b2MouseJointDef() {
    b2MouseJointDef.b2MouseJointDef.apply(this, arguments);
    if (this.constructor === b2MouseJointDef)
      this.b2MouseJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2MouseJointDef = b2MouseJointDef;

  function b2PrismaticJoint() {
    b2PrismaticJoint.b2PrismaticJoint.apply(this, arguments);
    if (this.constructor === b2PrismaticJoint)
      this.b2PrismaticJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2PrismaticJoint = b2PrismaticJoint;

  function b2PrismaticJointDef() {
    b2PrismaticJointDef.b2PrismaticJointDef.apply(this, arguments);
    if (this.constructor === b2PrismaticJointDef)
      this.b2PrismaticJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2PrismaticJointDef = b2PrismaticJointDef;

  function b2PulleyJoint() {
    b2PulleyJoint.b2PulleyJoint.apply(this, arguments);
    if (this.constructor === b2PulleyJoint)
      this.b2PulleyJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2PulleyJoint = b2PulleyJoint;

  function b2PulleyJointDef() {
    b2PulleyJointDef.b2PulleyJointDef.apply(this, arguments);
    if (this.constructor === b2PulleyJointDef)
      this.b2PulleyJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2PulleyJointDef = b2PulleyJointDef;

  function b2RevoluteJoint() {
    b2RevoluteJoint.b2RevoluteJoint.apply(this, arguments);
    if (this.constructor === b2RevoluteJoint)
      this.b2RevoluteJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2RevoluteJoint = b2RevoluteJoint;

  function b2RevoluteJointDef() {
    b2RevoluteJointDef.b2RevoluteJointDef.apply(this, arguments);
    if (this.constructor === b2RevoluteJointDef)
      this.b2RevoluteJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2RevoluteJointDef = b2RevoluteJointDef;

  function b2WeldJoint() {
    b2WeldJoint.b2WeldJoint.apply(this, arguments);
    if (this.constructor === b2WeldJoint)
      this.b2WeldJoint.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2WeldJoint = b2WeldJoint;

  function b2WeldJointDef() {
    b2WeldJointDef.b2WeldJointDef.apply(this, arguments);
    if (this.constructor === b2WeldJointDef)
      this.b2WeldJointDef.apply(this, arguments);
  }
  Box2D.Dynamics.Joints.b2WeldJointDef = b2WeldJointDef;
}
