b2TOIInput.b2TOIInput = function() {
  this.proxyA = new b2DistanceProxy();
  this.proxyB = new b2DistanceProxy();
  this.sweepA = new b2Sweep();
  this.sweepB = new b2Sweep();
};
