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
#include <map>
#include <iostream>
#include "imgui.h"
#include "graph.h"
#include "DebugDraw.h"


class QueryCallback : public b2QueryCallback {
public:
    QueryCallback(const b2Vec2 &point) {
        m_point = point;
        m_fixture = NULL;
    }

    bool ReportFixture(b2Fixture *fixture) {
        b2Body *body = fixture->GetBody();
        if (body->GetType() == b2_dynamicBody) {
            bool inside = fixture->TestPoint(m_point);
            if (inside) {
                m_fixture = fixture;

                // We are done, terminate the query.
                return false;
            }
        }

        // Continue the query.
        return true;
    }

    b2Vec2 m_point;
    b2Fixture *m_fixture;
};

// This is a fun demo that shows off the wheel joint
class Car : public Test {
public:


    struct boxBot {

        struct magnet {
            b2Vec2 pos;
            bool active = false;
        };
        int id;
        b2Body *box;
        b2Body *wheel;
        b2RevoluteJoint *spring;
        b2Fixture *fix;
        float buffer[100];
        magnet magnets[4];

    };


    boxBot *body2Bot(b2Body *b1) {
        for (std::vector<boxBot>::iterator bb = bots->begin(); bb != bots->end(); bb++) {
            if ((b1 == bb->box) || (b1 == bb->wheel)) {
                return &(*bb);
            }
        }
        return 0;


    };

    boxBot createBox(float x, float y, int id) {
        boxBot bb;

        bb.id=id;

        b2PolygonShape chassis;
        b2Vec2 vertices[4];

        vertices[0].Set(1.0f, -1.0f);
        bb.magnets[0].pos.Set(1.0f, -1.0f);
        bb.magnets[0].active = false;


        vertices[1].Set(1.0f, 1.0f);
        bb.magnets[1].pos.Set(1.0f, 1.0f);
        bb.magnets[1].active = false;

        vertices[2].Set(-1.0f, 1.0f);
        bb.magnets[2].pos.Set(-1.0f, 1.0f);
        bb.magnets[2].active = false;


        vertices[3].Set(-1.0f, -1.0f);
        bb.magnets[3].pos.Set(-1.0f, -1.0f);
        bb.magnets[3].active = false;




        chassis.Set(vertices, 4);

        b2CircleShape circle;
        circle.m_radius = 0.7f;

        b2BodyDef bd;

        bd.type = b2_dynamicBody;
        bd.position.Set(x, y);

        bb.box = m_world->CreateBody(&bd);

        b2FixtureDef fd0;
        fd0.shape = &chassis;
        fd0.density = 1.0f;
        fd0.friction = 0.9f;
        fd0.filter.groupIndex = 2;


        bb.fix = bb.box->CreateFixture(&fd0);


        b2FixtureDef fd;
        fd.shape = &circle;
        fd.density = 1.0f;
        fd.friction = 0.9f;
        fd.filter.groupIndex = 2;


        bd.position.Set(x, y);
        bb.wheel = m_world->CreateBody(&bd);
        bb.wheel->CreateFixture(&fd);


        b2RevoluteJointDef jd;
        b2Vec2 axis(0.0f, 1.0f);

        jd.Initialize(bb.box, bb.wheel, bb.wheel->GetPosition());
        jd.motorSpeed = 0.0f;
        jd.maxMotorTorque = 600.0f;
        jd.enableMotor = true;


        bb.spring = (b2RevoluteJoint *) m_world->CreateJoint(&jd);

        return bb;
    };

    Car() {

        m_speed = 100;

        b2Body *ground = NULL;
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

            float32 groundMap[10][2] = {{0.0f,0.0f}, {5.0f,0.0f}, {5.0f,3.0f}, {10.f,3.0f}, {15.0f,3.0f}, {20.f,3.0f},{20.f,6.0f}};

            float32 x = 20.0f, y1 = 0.0f, dx = 5.0f;
/*
            for (int32 i = 0; i < 10; ++i) {
                float32 y2 = hs[i];
                shape.Set(b2Vec2(x, y1), b2Vec2(x + dx, y2));
                ground->CreateFixture(&fd);
                y1 = y2;
                x += dx;
            }

*/          float x1 = groundMap[0][0];
            y1 = groundMap[0][1];
            for (int i = 1; i < 7; ++i) {
                float32 x2 = groundMap[i][0];
                float32 y2 = groundMap[i][1];
                shape.Set(b2Vec2(x1, y1), b2Vec2(x2, y2));
                ground->CreateFixture(&fd);
                y1 = y2;
                x1 = x2;
            }
        }




        //m_car=createBox(0.f,2.f);


        bots = new std::vector<boxBot>;
        bots->push_back(createBox(0.f, 9.f, 0));
        bots->push_back(createBox(0.f, 6.f, 1));
        bots->push_back(createBox(0.f, 2.f, 2));
        bots->push_back(createBox(-2.f, 2.f, 3));
        bots->push_back(createBox(2.f, 2.f, 3));
        //b1=createBox(0.f,2.f);

        //b2=createBox(2.f,2.f);



/*
		b2RevoluteJointDef jd;
		jd.collideConnected = true;


		jd.Initialize((*bots)[0].box, (*bots)[1].box, (*bots)[1].box->GetWorldCenter() + ((b2PolygonShape*)((*bots)[1].fix->GetShape()))->m_vertices[0]);


        magnetJoints = new std::vector<b2RevoluteJoint*>;
		magnetJoints->push_back((b2RevoluteJoint*)m_world->CreateJoint(&jd));
*/

        magnetJoints = new std::map<int,b2RevoluteJoint *>;
        currentBot = &((*bots)[0]);
    }

    int symmHash(short int a, short int b)
    {
        if (a>b)
            return symmHash(b,a);
        return  a << 16 |  b;
    };

    void updateJoints() {
        int id = 0;
        for (int i = 0; i < bots->size(); i++) {
            for (int j = i + 1; j < bots->size(); j++) {
                for (int k = 0; k < 4; k++) {
                    for (int l = 0; l < 4; l++) {

                        short int h1 =  i << 8 | k;
                        short int h2 =  j << 8 | l;
                        id = symmHash(h1,h2);


                        (*bots)[i].magnets[k].pos = (*bots)[i].box->GetWorldPoint(
                                ((b2PolygonShape *) ((*bots)[i].fix->GetShape()))->GetVertex(k));
                        (*bots)[j].magnets[l].pos = (*bots)[j].box->GetWorldPoint(
                                ((b2PolygonShape *) ((*bots)[j].fix->GetShape()))->GetVertex(l));

                        b2Vec2 dir = (*bots)[i].magnets[k].pos - (*bots)[j].magnets[l].pos;
                        float magn = dir.Length();
                        dir.Normalize();
                        //const b2Vec2 force = std::min(100/(magn*magn),100.f)*dir;
                        const b2Vec2 force = 200.f*std::exp(-20.f*magn*magn)*dir;
                        const b2Vec2 pos1 = (*bots)[i].magnets[k].pos;
                        const b2Vec2 pos2 = (*bots)[j].magnets[l].pos;

                        //const b2Vec2 force2 = -std::max(1/magn,10.f)*dir;

                        if (((*bots)[i].magnets[k].active) && ((*bots)[j].magnets[l].active)) {
                            (*bots)[j].box->ApplyForce(force,pos2, true);
                            (*bots)[i].box->ApplyForce(-force,pos1, true);


                            if (((*bots)[i].magnets[k].pos - (*bots)[j].magnets[l].pos).Length() < 0.02f) {
                                b2DistanceJointDef jd;


                                jd.collideConnected = true;
                                jd.length = 0.02f;
                                jd.dampingRatio=0.5f;
                                jd.Initialize((*bots)[i].box, (*bots)[j].box, (*bots)[i].magnets[k].pos,(*bots)[j].magnets[l].pos);
                                std::map<int,b2RevoluteJoint *>::iterator j1 = magnetJoints->find(id);
                                if (j1==magnetJoints->end()) {
                                    magnetJoints->insert(std::pair<int, b2RevoluteJoint *>(id, (b2RevoluteJoint *) m_world->CreateJoint(&jd)));
                                    ImGui::Text("magnet links active: %d", magnetJoints->size());
                                }

/*
                                ImGui::Text("%d",id);

                                b2RevoluteJointDef jd;
                                jd.collideConnected = true;
                                jd.Initialize((*bots)[i].box, (*bots)[j].box, (*bots)[i].magnets[k].pos);
                                std::map<int,b2RevoluteJoint *>::iterator j1 = magnetJoints->find(id);
                                if (j1==magnetJoints->end()) {
                                    magnetJoints->insert(std::pair<int, b2RevoluteJoint *>(id, (b2RevoluteJoint *) m_world->CreateJoint(&jd)));
                                    ImGui::Text("magnet links active: %d", magnetJoints->size());
                                }
*/
                                g_debugDraw.DrawPoint(pos1, 6, b2Color(0.f, 0.f, 1.f));
                            }



                        }
                        else{
                            std::map<int,b2RevoluteJoint *>::iterator j1 = magnetJoints->find(id);
                            if (j1!=magnetJoints->end()){

                                m_world->DestroyJoint(j1->second);
                                //m_world->DestroyJoint(magnetJoints->begin()->second);
                                magnetJoints->erase(id);
                                std::cout << "deleted" << id ;
                            }
                        }
                    }
                }
            }
        }
    }


    void Keyboard(int key) {
        switch (key) {
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


    void MouseDown(const b2Vec2 &p) {
        //Test::MouseDown(p);

        m_mouseWorld = p;


        if (m_mouseJoint != NULL) {
            return;
        }


        // Make a small box.
        b2AABB aabb;
        b2Vec2 d;
        d.Set(0.001f, 0.001f);
        aabb.lowerBound = p - d;
        aabb.upperBound = p + d;

        // Query the world for overlapping shapes.
        QueryCallback callback(p);
        m_world->QueryAABB(&callback, aabb);

        if (callback.m_fixture) {

            b2Body *body = callback.m_fixture->GetBody();

            currentBot = body2Bot(body);

            b2MouseJointDef md;
            md.bodyA = m_groundBody;
            md.bodyB = body;
            md.target = p;
            md.maxForce = 1000.0f * body->GetMass();
            m_mouseJoint = (b2MouseJoint *) m_world->CreateJoint(&md);
            body->SetAwake(true);

        }

        for (int i = 0; i < bots->size(); i++) {
            for (int j = 0; j < 4; j++) {
                if ((m_mouseWorld - (*bots)[i].magnets[j].pos).Length()<0.2f){
                    (*bots)[i].magnets[j].active=!(*bots)[i].magnets[j].active;
                }
            }
        }


    }



    void Step(Settings *settings) {

        g_camera.m_center.x = (*bots)[0].box->GetPosition().x;


        for (std::vector<boxBot>::iterator bb = bots->begin(); bb != bots->end(); bb++) {
            for (int i = 0; i < IM_ARRAYSIZE(bb->buffer); i++) {
                bb->buffer[i] = bb->buffer[(i + 1) % IM_ARRAYSIZE(bb->buffer)];
            }
            bb->buffer[IM_ARRAYSIZE(bb->buffer) - 1] = bb->box->GetPosition().x;
        }
        updateJoints();
        DrawMagnetScheme();
        DrawActiveMagnets();

        Test::Step(settings);
    }


    void plotGraphs() {


        ImVec2 foo[10];

        foo[0].x = 0; // init data so editor knows to take it from here

        for (int i = 0; i < 10; i++) {
            foo[i].x = i * 0.1f;
            foo[i].y = i * 0.1f;
        }


        float foo2[100];
        for (int i = 0; i < 100; i++) {
            foo2[i] = sinf(i * 0.1f);
        }

        bool f = true;
        ImGui::GraphTestWindow((*bots)[0].buffer, 100);

        ImGui::Curve("Curve", ImVec2(600, 200), 10, foo);

    }


    void DrawActiveMagnets() {
        for (int i = 0; i < bots->size(); i++) {
            for (int j = 0; j < 4; j++) {
                if ((*bots)[i].magnets[j].active) {
                    g_debugDraw.DrawPoint((*bots)[i].magnets[j].pos, 5, b2Color(1.f, 0.f, 0.f));
                }
            }
        }
    }

    void DrawMagnetScheme() {


        ImGui::SetNextWindowSize(ImVec2(250, 250), ImGuiSetCond_FirstUseEver);
        if (!ImGui::Begin("Magnets")) {


            ImGui::End();
            return;
        }

        ImGui::Text("current bot: %d", currentBot->id);


        ImDrawList *draw_list = ImGui::GetWindowDrawList();


        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();            // ImDrawList API uses screen coordinates!
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();        // Resize canvas to what's available

        draw_list->AddRectFilledMultiColor(canvas_pos,
                                           ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                                           ImColor(50, 50, 50), ImColor(50, 50, 60), ImColor(60, 60, 70),
                                           ImColor(50, 50, 60));
        draw_list->AddRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
                           ImColor(255, 255, 255));


        static float sz = 136.0f;
        static ImVec4 col = ImVec4(1.0f, 1.0f, 0.4f, 1.0f);
        const ImU32 col32 = ImColor(col);

        const ImVec2 p = ImGui::GetCursorScreenPos();

        ImVec2 mp = ImGui::GetIO().MousePos;
        float x = p.x + 40.0f, y = p.y + 40.0f;

        draw_list->AddRectFilled(ImVec2(x, y), ImVec2(x + sz, y + sz), col32);

        ImGui::InvisibleButton("canvas", canvas_size);
        if (ImGui::IsItemHovered()) {
            if (ImGui::IsMouseClicked(0)) {
                draw_list->AddRectFilled(ImVec2(mp.x - 5, mp.y - 5),
                                         ImVec2(mp.x + 5, mp.y + 5),
                                         ImColor(255, 0, 0));

                if (mp.x<p.x+sz/2){
                    if (mp.y>p.y+sz/2){
                        currentBot->magnets[3].active = not currentBot->magnets[3].active ;
                    } else{
                        currentBot->magnets[2].active = not currentBot->magnets[2].active ;
                    }
                } else {
                    if (mp.y > p.y + sz / 2) {
                        currentBot->magnets[0].active = not currentBot->magnets[0].active;
                    } else {
                        currentBot->magnets[1].active = not currentBot->magnets[1].active;
                    }
                }
            }
        }

        if (currentBot->magnets[3].active){
            draw_list->AddRectFilled(ImVec2(x - 5, y+sz - 5),
                                     ImVec2(x + 5, y+sz + 5),
                                     ImColor(255, 0, 0));
        }

        if (currentBot->magnets[2].active){
            draw_list->AddRectFilled(ImVec2(x - 5, y  - 5),
                                     ImVec2(x + 5, y  + 5),
                                     ImColor(255, 0, 0));
        }

        if (currentBot->magnets[1].active){
            draw_list->AddRectFilled(ImVec2(x + sz - 5, y  - 5),
                                     ImVec2(x + sz + 5, y  + 5),
                                     ImColor(255, 0, 0));
        }

        if (currentBot->magnets[0].active){
            draw_list->AddRectFilled(ImVec2(x + sz - 5, y + sz - 5),
                                     ImVec2(x + sz + 5, y + sz + 5),
                                     ImColor(255, 0, 0));
        }


        ImGui::End();



    }


    static Test *Create() {
        return new Car;
    }


    std::vector<boxBot> *bots;

    boxBot *currentBot;

    float32 m_speed;
    std::map<int,b2RevoluteJoint *> *magnetJoints;
};

#endif
