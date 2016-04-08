/*
* Copyright (c) 2006-2011 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#ifndef CAR_H
#define CAR_H

// This is a fun demo that shows off the wheel joint
class Car : public Test {
public:


	struct boxBot
	{
		b2Body* box;
		b2Body* wheel;
		b2RevoluteJoint* spring;
		b2Fixture* fix;
	};

	boxBot createBox(float x, float y)
	{
			boxBot bb;

			b2PolygonShape chassis;
			b2Vec2 vertices[4];
			vertices[0].Set(-1.0f, -1.0f);
			vertices[1].Set(-1.0f, 1.0f);
			vertices[2].Set(1.0f, 1.0f);
			vertices[3].Set(1.0f, -1.0f);
			chassis.Set(vertices, 4);

			b2CircleShape circle;
			circle.m_radius = 0.7f;

			b2BodyDef bd;

			bd.type = b2_dynamicBody;
			bd.position.Set(x,y);

			bb.box = m_world->CreateBody(&bd);
			bb.fix = bb.box->CreateFixture(&chassis, 1.0f);

			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;


			bd.position.Set(x,y);
			bb.wheel = m_world->CreateBody(&bd);
			bb.wheel->CreateFixture(&fd);


			b2RevoluteJointDef jd;
			b2Vec2 axis(0.0f, 1.0f);

			jd.Initialize(bb.box, bb.wheel, bb.wheel->GetPosition());
			jd.motorSpeed = 0.0f;
			jd.maxMotorTorque = 600.0f;
			jd.enableMotor = true;

			bb.spring = (b2RevoluteJoint*)m_world->CreateJoint(&jd);

			return bb;
	};

	Car()
	{

		m_speed = 50;

		b2Body* ground = NULL;
		{
			b2BodyDef bd;
			ground = m_world->CreateBody(&bd);

			b2EdgeShape shape;

			b2FixtureDef fd;
			fd.shape = &shape;
			fd.density = 0.0f;
			fd.friction = 0.9f;

			shape.Set(b2Vec2(-20.0f, 0.0f), b2Vec2(20.0f, 0.0f));

			ground->CreateFixture(&fd);

			float32 hs[10] = {0.25f, 1.0f, 4.0f, 0.0f, 0.0f, -1.0f, -2.0f, -2.0f, -1.25f, 0.0f};

			float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;

			for (int32 i = 0; i < 10; ++i)
			{
				float32 y2 = hs[i];
				shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
				ground->CreateFixture(&fd);
				y1 = y2;
				x += dx;
			}

		}

		//m_car=createBox(0.f,2.f);

		b1=createBox(0.f,2.f);

		b2=createBox(2.f,2.f);

		b2RevoluteJointDef jd;
		b2Vec2 axis(0.0f, 1.0f);

		jd.Initialize(b1.box, b2.box, ((b2PolygonShape*)(b1.fix->GetShape()))->m_vertices[0]);

		(b2RevoluteJoint*)m_world->CreateJoint(&jd);
	}



	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			b1.spring->SetMotorSpeed(-m_speed);
			break;

		case GLFW_KEY_D:
			b1.spring->SetMotorSpeed(m_speed);
			break;

		case GLFW_KEY_S:
			b1.spring->SetMotorSpeed(0.0f);
			break;

		}
	}

	void Step(Settings* settings)
	{

		g_camera.m_center.x = b1.box->GetPosition().x;
		Test::Step(settings);
	}

	static Test* Create()
	{
		return new Car;
	}


	boxBot b1;
	boxBot b2;


	float32 m_speed;
	b2RevoluteJoint* m_spring1;
};

#endif
