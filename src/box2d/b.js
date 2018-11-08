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
import * as Distance from './collision/distance';
import DistanceInput from './collision/distance-input';
import DistanceOutput from './collision/distance-output';
import DistanceProxy from './collision/distance-proxy';
import DynamicTree from './collision/dynamic-tree';
import DynamicTreeBroadPhase from './collision/dynamic-tree-broad-phase';
import DynamicTreeNode from './collision/dynamic-tree-node';
import DynamicTreePair from './collision/dynamic-tree-pair';
import Point from './collision/point';
import RayCastInput from './collision/ray-cast-input';
import RayCastOutput from './collision/ray-cast-output';
import Simplex from './collision/simplex';
import CircleShape from './collision/shapes/circle-shape';
import EdgeChainDef from './collision/shapes/edge-chain-def';
import EdgeShape from './collision/shapes/edge-shape';
import MassData from './collision/shapes/mass-data';
import PolygonShape from './collision/shapes/polygon-shape';
import Shape from './collision/shapes/shape';
import Segment from './collision/segment';
import SeparationFunction from './collision/separation-function';
import * as TimeOfImpact from './collision/time-of-impact';
import ToiInput from './collision/toi-input';
import WorldManifold from './collision/world-manifold';
import Body from './dynamics/body';
import BodyDef from './dynamics/body-def';
import Fixture from './dynamics/fixture';
import FilterData from './dynamics/filter-data';
import FixtureDef from './dynamics/fixture-def';
import ContactFilter from './dynamics/contact-filter';
import ContactImpulse from './dynamics/contact-impulse';
import ContactListener from './dynamics/contact-listener';
import ContactManager from './dynamics/contact-manager';
import World from './dynamics/world';
import CircleContact from './dynamics/contacts/circle-contact';
import ContactConstraintPoint from './dynamics/contacts/contact-constraint-point';
import ContactConstraint from './dynamics/contacts/contact-constraint';
import ContactEdge from './dynamics/contacts/contact-edge';
import ContactFactory from './dynamics/contacts/contact-factory';
import ContactRegister from './dynamics/contacts/contact-register';
import ContactResult from './dynamics/contacts/contact-result';
import ContactSolver from './dynamics/contacts/contact-solver';
import Contact from './dynamics/contacts/contact';
import EdgeAndCircleContact from './dynamics/contacts/edge-and-circle-contact';
import NullContact from './dynamics/contacts/null-contact';
import PolyAndCircleContact from './dynamics/contacts/poly-and-circle-contact';
import PolyAndEdgeContact from './dynamics/contacts/poly-and-edge-contact';
import PolygonContact from './dynamics/contacts/polygon-contact';
import PositionSolverManifold from './dynamics/contacts/position-solver-manifold';
import Island from './dynamics/island';
import Controller from './dynamics/controllers/controller';
import Joint from './dynamics/joints/joint';
import JointDef from './dynamics/joints/joint-def';
import JointEdge from './dynamics/joints/joint-edge';
import MouseJoint from './dynamics/joints/mouse-joint';
import MouseJointDef from './dynamics/joints/mouse-joint-def';
import DistanceJoint from './dynamics/joints/distance-joint';
import DistanceJointDef from './dynamics/joints/distance-joint-def';
import FrictionJoint from './dynamics/joints/friction-joint';
import FrictionJointDef from './dynamics/joints/friction-joint-def';
import GearJoint from './dynamics/joints/gear-joint';
import GearJointDef from './dynamics/joints/gear-joint-def';
import Jacobian from './dynamics/joints/jacobian';
import LineJoint from './dynamics/joints/line-joint';
import LineJointDef from './dynamics/joints/line-joint-def';
import PrismaticJoint from './dynamics/joints/prismatic-joint';
import PrismaticJointDef from './dynamics/joints/prismatic-joint-def';
import PulleyJoint from './dynamics/joints/pulley-joint';
import PulleyJointDef from './dynamics/joints/pulley-joint-def';
import RevoluteJoint from './dynamics/joints/revolute-joint';
import RevoluteJointDef from './dynamics/joints/revolute-joint-def';
import WeldJoint from './dynamics/joints/weld-joint';
import WeldJointDef from './dynamics/joints/weld-joint-def';

/**
 * @see: ./g.js
 */
// import DestructionListener from './dynamics/destruction-listener';

/**
 * @see: ./i.js
 */
// import BuoyancyController from './dynamics/controllers/buoyancy-controller';
// import ConstantAccelController from './dynamics/controllers/constant-accel-controller';
// import ConstantForceController from './dynamics/controllers/constant-force-controller';
// import ControllerEdge from './dynamics/controllers/controller-edge';
// import GravityController from './dynamics/controllers/gravity-controller';
// import TensorDampingController from './dynamics/controllers/tensor-damping-controller';

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
  Box2D.Collision.b2DynamicTree = DynamicTree;
  Box2D.Collision.b2DynamicTreeBroadPhase = DynamicTreeBroadPhase;
  Box2D.Collision.b2DynamicTreeNode = DynamicTreeNode;
  Box2D.Collision.b2DynamicTreePair = DynamicTreePair;
  Box2D.Collision.b2Manifold = Manifold;
  Box2D.Collision.b2ManifoldPoint = ManifoldPoint;
  Box2D.Collision.b2Point = Point;
  Box2D.Collision.b2RayCastInput = RayCastInput;
  Box2D.Collision.b2RayCastOutput = RayCastOutput;
  Box2D.Collision.b2Segment = Segment;
  Box2D.Collision.b2SeparationFunction = SeparationFunction;
  Box2D.Collision.b2Simplex = Simplex;
  Box2D.Collision.b2SimplexCache = SimplexCache;
  Box2D.Collision.b2SimplexVertex = SimplexVertex;
  Box2D.Collision.b2TimeOfImpact = TimeOfImpact;
  Box2D.Collision.b2TOIInput = ToiInput;
  Box2D.Collision.b2WorldManifold = WorldManifold;
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
  Box2D.Dynamics.b2Body = Body;
  Box2D.Dynamics.b2BodyDef = BodyDef;
  Box2D.Dynamics.b2ContactFilter = ContactFilter;
  Box2D.Dynamics.b2ContactImpulse = ContactImpulse;
  Box2D.Dynamics.b2ContactListener = ContactListener;
  Box2D.Dynamics.b2ContactManager = ContactManager;
  Box2D.Dynamics.b2DebugDraw = DebugDraw;

  function b2DestructionListener() {
    b2DestructionListener.b2DestructionListener.apply(this, arguments);
  }
  Box2D.Dynamics.b2DestructionListener = b2DestructionListener;

  Box2D.Dynamics.b2FilterData = FilterData;
  Box2D.Dynamics.b2Fixture = Fixture;
  Box2D.Dynamics.b2FixtureDef = FixtureDef;
  Box2D.Dynamics.b2Island = Island;
  Box2D.Dynamics.b2TimeStep = TimeStep;
  Box2D.Dynamics.b2World = World;
  Box2D.Dynamics.Contacts.b2CircleContact = CircleContact;
  Box2D.Dynamics.Contacts.b2Contact = Contact;
  Box2D.Dynamics.Contacts.b2ContactConstraint = ContactConstraint;
  Box2D.Dynamics.Contacts.b2ContactConstraintPoint = ContactConstraintPoint;
  Box2D.Dynamics.Contacts.b2ContactEdge = ContactEdge;
  Box2D.Dynamics.Contacts.b2ContactFactory = ContactFactory;
  Box2D.Dynamics.Contacts.b2ContactRegister = ContactRegister;
  Box2D.Dynamics.Contacts.b2ContactResult = ContactResult;
  Box2D.Dynamics.Contacts.b2ContactSolver = ContactSolver;
  Box2D.Dynamics.Contacts.b2EdgeAndCircleContact = EdgeAndCircleContact;
  Box2D.Dynamics.Contacts.b2NullContact = NullContact;
  Box2D.Dynamics.Contacts.b2PolyAndCircleContact = PolyAndCircleContact;
  Box2D.Dynamics.Contacts.b2PolyAndEdgeContact = PolyAndEdgeContact;
  Box2D.Dynamics.Contacts.b2PolygonContact = PolygonContact;
  Box2D.Dynamics.Contacts.b2PositionSolverManifold = PositionSolverManifold;

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

  Box2D.Dynamics.Controllers.b2Controller = Controller;

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

  Box2D.Dynamics.Joints.b2DistanceJoint = DistanceJoint;
  Box2D.Dynamics.Joints.b2DistanceJointDef = DistanceJointDef;
  Box2D.Dynamics.Joints.b2FrictionJoint = FrictionJoint;
  Box2D.Dynamics.Joints.b2FrictionJointDef = FrictionJointDef;
  Box2D.Dynamics.Joints.b2GearJoint = GearJoint;
  Box2D.Dynamics.Joints.b2GearJointDef = GearJointDef;
  Box2D.Dynamics.Joints.b2Jacobian = Jacobian;
  Box2D.Dynamics.Joints.b2Joint = Joint;
  Box2D.Dynamics.Joints.b2JointDef = JointDef;
  Box2D.Dynamics.Joints.b2JointEdge = JointEdge;
  Box2D.Dynamics.Joints.b2LineJoint = LineJoint;
  Box2D.Dynamics.Joints.b2LineJointDef = LineJointDef;
  Box2D.Dynamics.Joints.b2MouseJoint = MouseJoint;
  Box2D.Dynamics.Joints.b2MouseJointDef = MouseJointDef;
  Box2D.Dynamics.Joints.b2PrismaticJoint = PrismaticJoint;
  Box2D.Dynamics.Joints.b2PrismaticJointDef = PrismaticJointDef;
  Box2D.Dynamics.Joints.b2PulleyJoint = PulleyJoint;
  Box2D.Dynamics.Joints.b2PulleyJointDef = PulleyJointDef;
  Box2D.Dynamics.Joints.b2RevoluteJoint = RevoluteJoint;
  Box2D.Dynamics.Joints.b2RevoluteJointDef = RevoluteJointDef;
  Box2D.Dynamics.Joints.b2WeldJoint = WeldJoint;
  Box2D.Dynamics.Joints.b2WeldJointDef = WeldJointDef;
}
