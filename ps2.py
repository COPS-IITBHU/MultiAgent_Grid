from Box2D.Box2D import b2PolygonShape
from Box2D.examples.framework import (Framework, Keys, main)
from Box2D import (b2CircleShape, b2EdgeShape, b2FixtureDef, b2PolygonShape,
                   b2Transform, b2Mul,
                   b2_pi)
from Box2D import *

class ps2Arena (Framework):
    #pybox follows normal 2d sign conventions
    #arena has been drawn to scale

    def __init__(self):
        super(ps2Arena, self).__init__()

        # upper lower wall
        ground = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 7*6*0.0254 ), (7*6*0.0254, 7*6*0.0254)])
        )
        ground1 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -7*6*0.0254 ), (7*6*0.0254, -7*6*0.0254)])
        )
        # middle
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 1*6*0.0254 ), (1*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -1*6*0.0254 ), (1*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, 1*6*0.0254 ), (1*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 1*6*0.0254 ), (-1*6*0.0254, -1*6*0.0254)])
        )
        # middle top
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 5*6*0.0254 ), (1*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 3*6*0.0254 ), (1*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, 5*6*0.0254 ), (1*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, 5*6*0.0254 ), (-1*6*0.0254, 3*6*0.0254)])
        )
        # middle low
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -5*6*0.0254 ), (1*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -3*6*0.0254 ), (1*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(1*6*0.0254, -5*6*0.0254 ), (1*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-1*6*0.0254, -5*6*0.0254 ), (-1*6*0.0254, -3*6*0.0254)])
        )
        # right left
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 1*6*0.0254 ), (5*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -1*6*0.0254 ), (5*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 1*6*0.0254 ), (3*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, 1*6*0.0254 ), (5*6*0.0254, -1*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 1*6*0.0254 ), (-5*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -1*6*0.0254 ), (-5*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 1*6*0.0254 ), (-3*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, 1*6*0.0254 ), (-5*6*0.0254, -1*6*0.0254)])
        )
        # corner
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 5*6*0.0254 ), (5*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 3*6*0.0254 ), (5*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, 5*6*0.0254 ), (3*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, 5*6*0.0254 ), (5*6*0.0254, 3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 5*6*0.0254 ), (-5*6*0.0254, 5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 3*6*0.0254 ), (-5*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, 5*6*0.0254 ), (-3*6*0.0254, 3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, 5*6*0.0254 ), (-5*6*0.0254, 3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -5*6*0.0254 ), (5*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -3*6*0.0254 ), (5*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(3*6*0.0254, -5*6*0.0254 ), (3*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(5*6*0.0254, -5*6*0.0254 ), (5*6*0.0254, -3*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -5*6*0.0254 ), (-5*6*0.0254, -5*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -3*6*0.0254 ), (-5*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-3*6*0.0254, -5*6*0.0254 ), (-3*6*0.0254, -3*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-5*6*0.0254, -5*6*0.0254 ), (-5*6*0.0254, -3*6*0.0254)])
        )
        # side wall
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 7*6*0.0254 ), (7*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 7*6*0.0254 ), (-7*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -7*6*0.0254 ), (7*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -7*6*0.0254 ), (-7*6*0.0254, -2*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 1*6*0.0254 ), (7*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 1*6*0.0254 ), (-7*6*0.0254, -1*6*0.0254)])
        )
        # chute
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(8*6*0.0254, 1*6*0.0254 ), (8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-8*6*0.0254, 1*6*0.0254 ), (-8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(8*6*0.0254, -1*6*0.0254 ), (8*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-8*6*0.0254, -1*6*0.0254 ), (-8*6*0.0254, -2*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 1*6*0.0254 ), (8*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 1*6*0.0254 ), (-8*6*0.0254, 1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -1*6*0.0254 ), (8*6*0.0254, -1*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -1*6*0.0254 ), (-8*6*0.0254, -1*6*0.0254)])
        )

        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, 2*6*0.0254 ), (8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, 2*6*0.0254 ), (-8*6*0.0254, 2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(7*6*0.0254, -2*6*0.0254 ), (8*6*0.0254, -2*6*0.0254)])
        )
        ground3 = self.world.CreateStaticBody(
            shapes=b2EdgeShape(vertices=[(-7*6*0.0254, -2*6*0.0254 ), (-8*6*0.0254, -2*6*0.0254)])
        )
        
        # ground0 = self.world.CreateStaticBody(
        #     position=(0,7*6*0.0254),
        #     shapes=b2PolygonShape(box=(14*6*0.0254, 0.0254/4))
        # )
        # ground1 = self.world.CreateStaticBody(
        #     position=(0,-7*6*0.0254),
        #     shapes=b2PolygonShape(box=(14*6*0.0254, 0.0254/4))
        # )
        # ground2 = self.world.CreateStaticBody(
        #     position=(7*6*0.0254,0),
        #     shapes=b2PolygonShape(box=( 0.0254/4, 14*6*0.0254))
        # )
        # ground3 = self.world.CreateStaticBody(
        #     position=(-7*6*0.0254, 0),
        #     shapes=b2PolygonShape(box=( 0.0254/4,14*6*0.0254))
        # )

        # radius = 1.0
        # density = 1.0
        # # The bodies
        # for i, restitution in enumerate([0.0, 0.1, 0.3, 0.5, 0.75, 0.9, 1.0]):
        #     self.world.CreateDynamicBody(
        #         position=(-10 + 3.0 * i, 20),
        #         fixtures=b2FixtureDef(
        #             shape=b2CircleShape(radius=radius),
        #             density=density, restitution=restitution)
        #     )


        radius = 2*0.0254
        density = 0.1
        restitution = 0.0
        self.body = self.world.CreateDynamicBody(position=(0, 0), fixtures=b2FixtureDef(shape=b2CircleShape(radius=radius), density=density, restitution=restitution))

    def Keyboard(self, key):
        if not self.body:
            return

        if key == Keys.K_w:
            f = self.body.GetWorldVector(localVector=(0.0, -0.005))
            p = self.body.GetWorldPoint(localPoint=(0.0, 0.005))
            self.body.ApplyForce(f, p, True)
        elif key == Keys.K_a:
            self.body.ApplyTorque(0.0001, True)
        elif key == Keys.K_d:
            self.body.ApplyTorque(-0.0001, True)
        elif key == Keys.K_s:
            f = self.body.GetWorldVector(localVector=(0.0, -0.005))
            p = self.body.GetWorldPoint(localPoint=(0.0, -0.005))
            self.body.ApplyForce(f, p, True)

if __name__ == "__main__":
    main(ps2Arena)