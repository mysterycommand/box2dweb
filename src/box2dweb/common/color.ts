function parseUInt(v: number) {
  return Math.abs(parseInt(`${v}`, 10));
}

/**
 * @see: ./f.js#L314
 */
function Clamp(a = 0, low = 0, high = 0) {
  return a < low ? low : a > high ? high : a;
}

export default class Color {
  public get r() {
    return this.$r;
  }

  public set r(val: number) {
    this.$r = parseUInt(255 * Clamp(val, 0.0, 1.0));
  }

  public get g() {
    return this.$g;
  }

  public set g(val: number) {
    this.$g = parseUInt(255 * Clamp(val, 0.0, 1.0));
  }

  public get b() {
    return this.$b;
  }

  public set b(val: number) {
    this.$b = parseUInt(255 * Clamp(val, 0.0, 1.0));
  }

  public get color() {
    return (this.$r << 16) | (this.$g << 8) | this.$b;
  }

  private $r = 0;
  private $g = 0;
  private $b = 0;

  constructor(r = 0, g = 0, b = 0) {
    this.Set(r, g, b);
  }

  public Set(r = 0, g = 0, b = 0) {
    this.r = r;
    this.g = g;
    this.b = b;
  }
}
