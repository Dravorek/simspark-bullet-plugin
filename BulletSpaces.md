# In ODE #

Spaces in ODE are non-placable boundaries for a collection of collision objects and spaces. The hierarchies that you build with them speed up the collision algorithm in ODE.


# In Bullet #

Bullet uses pluggable Broadphase and Narrowphase collision handlers. These use different internal Structures to speed up collision detection, so organizing objects in Spaces doesn't help Bullet in general if we want to keep the option of choosing a different broadphase and narrowphase handlers.
We still need to generate our Collision objects in order to feed the Data to the sensors. If I can't get it out of the handlers in General, I might have to keep a btCollisionWorld around just for the sensors but I'll try to find another solution first.

The current status is that BulletSpace.cpp is mostly a dummy and the Collider::OnCollision  method gets called by the seperate bulletcollisionhandler.cpp which is hooked into bullet as an action. This solution is not optimal but it aims to avoid computing the collisions a second time after narrowphase.