package math;

public class Vec3 {
    public float x, y, z;

    public static Vec3 sampleOnSphere(float radius) {
        return new Vec3((float) Math.random() * 2 - 1, (float) Math.random() * 2 - 1, (float) Math.random() * 2 - 1).unit().scale(radius);
    }

    public static Vec3 zero() {
        return new Vec3();
    }

    public static Vec3 of(float c) {
        return new Vec3(c, c, c);
    }

    public static Vec3 of(float x, float y, float z) {
        return new Vec3(x, y, z);
    }

    public static Vec3 of(double x, double y, double z) {
        return new Vec3((float) x, (float) y, (float) z);
    }

    public boolean equals(Vec3 b) {
        return this.x == b.x && this.y == b.y && this.z == b.z;
    }

    public Vec3 plus(Vec3 b) {
        return new Vec3(this.x + b.x, this.y + b.y, this.z + b.z);
    }
    
    public Vec3 plusAccumulate(Vec3 b) {
        this.x += b.x;
        this.y += b.y;
        this.z += b.z;
        return this;
    }

    public Vec3 minus(Vec3 b) {
        return new Vec3(this.x - b.x, this.y - b.y, this.z - b.z);
    }
    
    public Vec3 minusAccumulate(Vec3 b) {
        this.x -= b.x;
        this.y -= b.y;
        this.z -= b.z;
        return this;
    }

    public Vec3 scale(float t) {
        return new Vec3(this.x * t, this.y * t, this.z * t);
    }

    public Vec3 scaleAccumulate(float t) {
        this.x  *= t;
        this.y  *= t;
        this.z  *= t;
        return this;
    }

    public float dot(Vec3 b) {
        return this.x * b.x + this.y * b.y + this.z * b.z;
    }

    public Vec3 cross(Vec3 b) {
        return new Vec3(this.y * b.z - this.z * b.y, this.z * b.x - this.x * b.z, this.x * b.y - this.y * b.x);
    }

    public float abs() {
        return (float) Math.sqrt(this.x * this.x + this.y * this.y + this.z * this.z);
    }

    public Vec3 unit() {
        float abs = this.abs();
        if (abs < 1e-6f) {
            return new Vec3(this);
        } else {
            return new Vec3(this).scale(1 / abs);
        }
    }

    public String toString() {
        return "V3D (" + x + ", " + y + ", " + z + ")";
    }

    private Vec3() {
        x = y = z = 0;
    }

    private Vec3(float x, float y, float z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    private Vec3(Vec3 c) {
        this.x = c.x;
        this.y = c.y;
        this.z = c.z;
    }

	public void minus(float x, float y, float z1) {
		this.x -= x;
		this.y -= y;
		this.z -= z1;
	}
	public void plus(float x, float y, float z) {
		this.x += x;
		this.y += y;
		this.z += z;
	}

	public Vec3 copy() {
		return new Vec3(this.x, this.y, this.z);
	}

	public void set(Vec3 vector) {
		this.x = vector.x;
		this.y = vector.y;
		this.z = vector.z;
	}
}
