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

#include <vector>
#include <iostream>
#include "imgui.h"
#include "graph.h"

class QueryCallback : public b2QueryCallback
{
public:
	QueryCallback(const b2Vec2& point)
	{
		m_point = point;
		m_fixture = NULL;
	}

	bool ReportFixture(b2Fixture* fixture)
	{
		b2Body* body = fixture->GetBody();
		if (body->GetType() == b2_dynamicBody)
		{
			bool inside = fixture->TestPoint(m_point);
			if (inside)
			{
				m_fixture = fixture;

				// We are done, terminate the query.
				return false;
			}
		}

		// Continue the query.
		return true;
	}

	b2Vec2 m_point;
	b2Fixture* m_fixture;
};

// This is a fun demo that shows off the wheel joint
class Car : public Test {
public:


	struct boxBot
	{
		b2Body* box;
		b2Body* wheel;
		b2RevoluteJoint* spring;
		b2Fixture* fix;
		float buffer[100];
	};


    boxBot* body2Bot(b2Body* b1)
    {
        for (std::vector <boxBot>::iterator bb = bots->begin(); bb!= bots->end();bb++) {
            if ((b1 == bb->box) || (b1 == bb->wheel))
            {
                return &(*bb);
            }
        }
        return 0;


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

			b2FixtureDef fd0;
			fd0.shape = &chassis;
			fd0.density = 1.0f;
			fd0.friction = 0.9f;
			fd0.filter.groupIndex=2;



			bb.fix = bb.box->CreateFixture(&fd0);


			b2FixtureDef fd;
			fd.shape = &circle;
			fd.density = 1.0f;
			fd.friction = 0.9f;
			fd.filter.groupIndex=2;


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


		bots = new std::vector<boxBot>;
		bots->push_back(createBox(2.f,2.f));
		bots->push_back(createBox(0.f,2.f));
		//b1=createBox(0.f,2.f);

		//b2=createBox(2.f,2.f);

		b2RevoluteJointDef jd;
		jd.collideConnected = true;


		jd.Initialize((*bots)[0].box, (*bots)[1].box, (*bots)[1].box->GetWorldCenter() + ((b2PolygonShape*)((*bots)[1].fix->GetShape()))->m_vertices[0]);


        magnets = new std::vector<b2RevoluteJoint*>;
		magnets->push_back((b2RevoluteJoint*)m_world->CreateJoint(&jd));


		currentBot=&((*bots)[0]);
	}



	void Keyboard(int key)
	{
		switch (key)
		{
		case GLFW_KEY_A:
			currentBot->spring->SetMotorSpeed(-m_speed);
			break;

		case GLFW_KEY_D:
			currentBot->spring->SetMotorSpeed(m_speed);
			break;

		case GLFW_KEY_S:
			currentBot->spring->SetMotorSpeed(0.0f);
			break;

		}
	}

    //boxBot* body2bot(b2)


	void MouseDown(const b2Vec2& p)
	{

		m_mouseWorld = p;

		// Make a small box.
		b2AABB aabb;
		b2Vec2 d;
		d.Set(0.001f, 0.001f);
		aabb.lowerBound = p - d;
		aabb.upperBound = p + d;

		// Query the world for overlapping shapes.
		QueryCallback callback(p);
		m_world->QueryAABB(&callback, aabb);

		if (callback.m_fixture)
		{

			b2Body* body = callback.m_fixture->GetBody();

            currentBot = body2Bot(body);

			//currentBot = body;

			b2Transform transform;

            transform.Set(p,0.f);

			b2Vec2 point(currentBot->box->GetWorldCenter() + ((b2PolygonShape*)(currentBot->fix->GetShape()))->m_vertices[0]);

			b2CircleShape s1;
			s1.m_radius = 0.2f;

			bool hit = s1.TestPoint(transform, point);
            std::cout << hit;

            if (hit)
            {
                m_world->DestroyJoint((*magnets)[0]);
                (*magnets).pop_back();

                return;
            }

		}


	}


	void Step(Settings* settings)
	{

		g_camera.m_center.x = (*bots)[0].box->GetPosition().x;


        for (std::vector <boxBot>::iterator bb = bots->begin(); bb!= bots->end();bb++) {
            for (int i=0; i<IM_ARRAYSIZE(bb->buffer);i++) {
                bb->buffer[i] = bb->buffer[(i+1) % IM_ARRAYSIZE(bb->buffer)];
            }
            bb->buffer[IM_ARRAYSIZE(bb->buffer)-1] = bb->box->GetPosition().x;
        }

		Test::Step(settings);
	}


    void plotGraphs()
    {


        ImVec2 foo[10];

        foo[0].x = 0; // init data so editor knows to take it from here

        for (int i=0;i<10;i++)
        {
            foo[i].x = i*0.1f;
            foo[i].y = i*0.1f;
        }


        float foo2[100];
        for (int i=0;i<100;i++)
        {
            foo2[i]=sinf(i*0.1f);
        }

        bool f = true;
        ImGui::GraphTestWindow( (*bots)[0].buffer,100);

        ImGui::Curve("Curve", ImVec2(600, 200), 10, foo);

    }

	static Test* Create()
	{
		return new Car;
	}




	std::vector <boxBot> *bots;

	boxBot* currentBot;

	float32 m_speed;
	std::vector <b2RevoluteJoint*> *magnets;
};

#endif
