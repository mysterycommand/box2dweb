import Transform from '../common/math/transform';
import DistanceProxy from './distance-proxy';

export default class DistanceInput {
  public proxyA = new DistanceProxy();
  public proxyB = new DistanceProxy();

  public transformA = new Transform();
  public transformB = new Transform();

  public useRadii = false;
}
