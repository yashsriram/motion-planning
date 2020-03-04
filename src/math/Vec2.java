package math;

public class Vec2 {
    public float x, y;

    public static Vec2 sampleOnSphere(float radius) {
        return new Vec2((float) Math.random() * 2 - 1, (float) Math.random() * 2 - 1);
    }

    public static Vec2 zero() {
        return new Vec2();
    }

    public static Vec2 of(float c) {
        return new Vec2(c, c);
    }

    public static Vec2 of(float x, float y) {
        return new Vec2(x, y);
    }

    public static Vec2 of(double x, double y) {
        return new Vec2((float) x, (float) y);
    }

    public boolean equals(Vec2 b) {
        return this.x == b.x && this.y == b.y;
    }

    public Vec2 plus(Vec2 b) {
        return new Vec2(this.x + b.x, this.y + b.y);
    }

    public Vec2 plusAccumulate(Vec2 b) {
        this.x += b.x;
        this.y += b.y;
        return this;
    }

    public Vec2 minus(Vec2 b) {
        return new Vec2(this.x - b.x, this.y - b.y);
    }

    public Vec2 minusAccumulate(Vec2 b) {
        this.x -= b.x;
        this.y -= b.y;
        return this;
    }

    public Vec2 scale(float t) {
        return new Vec2(this.x * t, this.y * t);
    }

    public Vec2 scaleAccumulate(float t) {
        this.x  *= t;
        this.y  *= t;
        return this;
    }

    public float dot(Vec2 b) {
        return this.x * b.x + this.y * b.y;
    }

    public float abs() {
        return (float) Math.sqrt(this.x * this.x + this.y * this.y);
    }

    public Vec2 unit() {
        float abs = this.abs();
        if (abs < 1e-6f) {
            return new Vec2(this);
        } else {
            return new Vec2(this).scale(1 / abs);
        }
    }

    public String toString() {
        return "V2D (" + x + ", " + y + ")";
    }

    private Vec2() {
        x = 0;
        y = 0;
    }

    private Vec2(float x, float y) {
        this.x = x;
        this.y = y;
    }

    private Vec2(Vec2 c) {
        this.x = c.x;
        this.y = c.y;
    }
}
