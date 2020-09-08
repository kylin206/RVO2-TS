import Vector2D from "./Vector2D";

export default class Line {
    /**
     * Constructs and initializes a directed line with the specified point and
     * direction.
     *
     * @param point     A point on the directed line.
     * @param direction The direction of the directed line.
     */
    constructor(point: Vector2D = null, direction: Vector2D = null) {
        this.direction = direction;
        this.point = point;
    }

    /**
     * The direction of this directed line.
     */
    public direction: Vector2D = Vector2D.ZERO;

    /**
     * A point on this directed line.
     */
    public point: Vector2D = Vector2D.ZERO;

}
