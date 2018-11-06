// tslint:disable variable-name

export default class Settings {
  public static VERSION = '2.1alpha';
  public static USHRT_MAX = 0x0000ffff;
  public static b2_pi = Math.PI;
  public static b2_maxManifoldPoints = 2;
  public static b2_aabbExtension = 0.1;
  public static b2_aabbMultiplier = 2.0;
  public static b2_linearSlop = 0.005;
  public static b2_polygonRadius = 2.0 * Settings.b2_linearSlop;
  public static b2_angularSlop = (2.0 / 180.0) * Settings.b2_pi;
  public static b2_toiSlop = 8.0 * Settings.b2_linearSlop;
  public static b2_maxTOIContactsPerIsland = 32;
  public static b2_maxTOIJointsPerIsland = 32;
  public static b2_velocityThreshold = 1.0;
  public static b2_maxLinearCorrection = 0.2;
  public static b2_maxAngularCorrection = (8.0 / 180.0) * Settings.b2_pi;
  public static b2_maxTranslation = 2.0;
  public static b2_maxTranslationSquared =
    Settings.b2_maxTranslation * Settings.b2_maxTranslation;
  public static b2_maxRotation = 0.5 * Settings.b2_pi;
  public static b2_maxRotationSquared =
    Settings.b2_maxRotation * Settings.b2_maxRotation;
  public static b2_contactBaumgarte = 0.2;
  public static b2_timeToSleep = 0.5;
  public static b2_linearSleepTolerance = 0.01;
  public static b2_angularSleepTolerance = (2.0 / 180.0) * Settings.b2_pi;

  public static b2MixFriction(friction1 = 0, friction2 = 0) {
    return Math.sqrt(friction1 * friction2);
  }

  public static b2MixRestitution(restitution1 = 0, restitution2 = 0) {
    return restitution1 > restitution2 ? restitution1 : restitution2;
  }

  public static b2Assert(a: boolean) {
    if (!a) {
      throw new Error('Assertion failed!');
    }
  }
}
