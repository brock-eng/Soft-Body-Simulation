#include <vector>

#include "LegitEngineCore/legit_engine.h"

#define CREATE_MESH testMesh = CreateMesh(200, 200, m_ScreenWidth / 2.0f, m_ScreenHeight / 2.0f, 5, 5);
#define MAXIMUM(a,b) (((a) > (b)) ? (a) : (b))
#define MINIMUM(a,b) (((a) < (b)) ? (a) : (b))
#define SIGN(x)      (((x) < 0) ? -1 : 1)

using namespace legit_engine;
using namespace components;
using namespace renderables;

using namespace std;

float globalDampingFactor = 0.200f;
float globalSpringStiffness = 250.0f;
float globalNodeRadius = 7.0f;
float globalNodeMass = 0.5f;
float globalSpringThickness = 1.0f;
float gravity = 300.0f;
float fVelocityFactor = 2.0f;
float radiusSelectFactor = 2.0f;
float globalLineRadius = 10.0f;
float dynamicSpringFactor = 1.0f;

int numIterations = 4;

class SoftbodySim : public Application
{
   Texture circleTexture = Texture("res/circle.png");

   struct Node
   {
      Node(float x, float y)
         : pX(x), pY(y) {}

      float pX, pY;
      float vX = 0, vY = 0;
      float fX = 0, fY = 0;

      float mass = globalNodeMass;
      float radius = globalNodeRadius;
   };

   struct Spring
   {
      Spring(Node* _A, Node* _B) : A(_A), B(_B) {}

      Node* A, *B;                        // nodes at end of each spring
      float L;                            // rest length
      float kS = globalSpringStiffness;   // stiffness
      float kD = globalDampingFactor;     // damping factor

      // returns the current spring length
      float Length()
      { 
         return sqrt((A->pX - B->pX) * (A->pX - B->pX) + (A->pY - B->pY) * (A->pY - B->pY));
      }
   };

   struct Mesh
   {
      vector<Node*> Nodes;
      vector<Spring*> Springs;

      float width, height;
   };

   struct Collision
   {
      Node* n1;
      Node* n2;

      Collision(Node* node1, Node* node2) : n1(node1), n2(node2) {}
   };

   struct LineSegment
   {
      float sx, sy;
      float ex, ey;
      float radius;

      bool operator==(LineSegment line)
      {
         return (sx == line.sx && sy == line.sy && ex == line.ex && ey == line.ey);
      }
   };

   // Creates a mesh object with a specified width and height 
   // and starting position at the bottom left corner
   Mesh* CreateMesh(float width, float height, float xPos, float yPos, int numX, int numY)
   {
      Mesh* newMesh = new Mesh;

      newMesh->width = width;
      newMesh->height = height;

      newMesh->Nodes.reserve((int)(width / numX * height / numY));
      newMesh->Springs.reserve((numX - 1) * (numY - 1) * 3 + numX + numY - 2);

      /*--- Nodes ---*/
      for (int y = 0; y < numY; y++)
      {
         for (int x = 0; x < numX; x++)
         {
            newMesh->Nodes.push_back(new Node((float)(x + 1) / numX * width + xPos, (float)(y + 1) / numY * height + yPos));
         }
      }

      /*--- Springs ---*/
      // Bottom
      for (int i = 0; i < numX - 1; i++)
      {
         newMesh->Springs.push_back(new Spring(newMesh->Nodes[i], newMesh->Nodes[i + 1]));
      }
      // Left column
      for (int i = 0; i < numY - 1; i++)
      {
         newMesh->Springs.push_back(new Spring(newMesh->Nodes[i * numX], newMesh->Nodes[(i + 1) * numX]));
      }
      // Middle Nodes
      for (int i = newMesh->Nodes.size() - 1; i >= numX; i--)
      {
         if (i % numX != 0)
         {
            newMesh->Springs.push_back(new Spring(newMesh->Nodes[i], newMesh->Nodes[i - 1]));
            newMesh->Springs.push_back(new Spring(newMesh->Nodes[i], newMesh->Nodes[(i - 1) - numX]));
            newMesh->Springs.push_back(new Spring(newMesh->Nodes[i], newMesh->Nodes[i - numX]));
         }
      }

      // Adding 
      for (int i = numX - 1; i < newMesh->Nodes.size() - 1; i++)
      {
         if ((i + 1) % (numX) != 0)
         {
            newMesh->Springs.push_back(new Spring(newMesh->Nodes[i], newMesh->Nodes[(i + 1) - numX]));
         }
      }

      // Setting rest spring length
      for (auto& spring : newMesh->Springs)
      {
         spring->L = spring->Length();
      }

      return newMesh;
   }

   void RenderMesh(Mesh* mesh)
   {
      for (auto& spring : mesh->Springs)
      {
         RenderLine(spring->A->pX, spring->A->pY, spring->B->pX, spring->B->pY, 99999999, 1.0f);
      }

      for (auto& node : mesh->Nodes)
      {
         RenderQuad(node->pX, node->pY, globalNodeRadius, globalNodeRadius, 0, &circleTexture);
      }
   }

   void CalculateSpringForces(Mesh* mesh)
   {
      float deltaL;
      float theta;
      float forceX, forceY;
      float dampX, dampY;

      // Reset spring forces to zero for each iteration
      for (auto& spring : mesh->Springs)
      {
         spring->A->fX = 0;
         spring->A->fY = 0;
         spring->B->fX = 0;
         spring->B->fY = 0;
      }
      
      // Go through each spring and add calculated force components to spring nodes
      for (auto& spring : mesh->Springs)
      {
         // Finding the angle made between the spring and the horizontal axis
         if ( spring->B->pX > spring->A->pX)
            theta = atanf((spring->B->pY - spring->A->pY) / (spring->B->pX - spring->A->pX));
         else
            theta = atanf((spring->B->pY - spring->A->pY) / (spring->B->pX - spring->A->pX)) + PI;

         // F = k * dx
         deltaL = spring->Length() - spring->L;

         // Applying a dynamic spring effect if set
         forceX = globalSpringStiffness * SIGN(deltaL) * pow(abs(deltaL), dynamicSpringFactor) * cosf(theta);
         forceY = globalSpringStiffness * SIGN(deltaL) * pow(abs(deltaL), dynamicSpringFactor) * sinf(theta);


         // Damping
         dampX = (spring->B->vX - spring->A->vX) * globalDampingFactor;
         dampY = (spring->B->vY - spring->A->vY) * globalDampingFactor;

         // Applying final forces
         spring->A->fX += forceX + dampX;
         spring->A->fY += forceY + dampY;

         spring->B->fX -= forceX + dampX;
         spring->B->fY -= forceY + dampY;
      }
   }

   void CalculateGravitationalForces(Mesh* mesh)
   {
      for (auto& node : mesh->Nodes)
      {
         node->fY -= gravity * node->mass;
      }
   }

   void CalculateLineNormal(Mesh* mesh)
   {
      for (auto& node1 : mesh->Nodes)
      {
         // checking for collisions on the line
         for (LineSegment* line : lineSegments)
         {
            float fLineX1 = line->ex - line->sx;
            float fLineY1 = line->ey - line->sy;

            float fLineX2 = node1->pX - line->sx;
            float fLineY2 = node1->pY - line->sy;

            float fEdgeLength = fLineX1 * fLineX1 + fLineY1 * fLineY1;

            float t = MAXIMUM(0, MINIMUM(fEdgeLength, (fLineX1 * fLineX2 + fLineY1 * fLineY2))) / fEdgeLength;
            float fClosestPointX = line->sx + t * fLineX1;
            float fClosestPointY = line->sy + t * fLineY1;

            float fDistance = sqrtf((node1->pX - fClosestPointX) * (node1->pX - fClosestPointX) +
               (node1->pY - fClosestPointY) * (node1->pY - fClosestPointY));

            // Checking for overlap
            if (fDistance <= (node1->radius + globalLineRadius))
            {
               Node temp(fClosestPointX, fClosestPointY);

               float fOverlap = 1.0f * (fDistance - node1->radius - globalLineRadius);

               node1->pX -= fOverlap * (node1->pX - temp.pX) / fDistance;
               node1->pY -= fOverlap * (node1->pY - temp.pY) / fDistance;


               
               if (normalForce)
               {

                  // Line angle to horizontal
                  float theta;

                  if (line->ex > line->sx)
                     theta = atanf((line->ey - line->sy) / (line->ex - line->sx));
                  else
                     theta = atanf((line->ey - line->sy) / (line->ex - line->sx)) + PI;

                  // Add normal force
                  float normal = 0;

                  float cosine = cos(theta);
                  float sine = sin(theta);

                  float appliedObjectYForce = cosine * node1->fY;
                  (appliedObjectYForce > 0) ? appliedObjectYForce : 0;

                  normal = cosine * node1->mass * gravity - appliedObjectYForce + sine * node1->fX;

                  node1->fX -= normal * sine;
                  node1->fY += normal * cosine;
               }
            }
         }
      }
   }

   void CalculateNodeVelocity(Mesh* mesh, float fDeltaTime)
   {
      float accelX, accelY;

      for (auto& node : mesh->Nodes)
      {
         accelX = (node->fX / node->mass);
         accelY = (node->fY / node->mass);

         node->vX += accelX * fDeltaTime * fVelocityFactor;
         node->vY += accelY * fDeltaTime * fVelocityFactor;

         if (std::abs(node->vX) < 0.01f) node->vX = 0;
         if (std::abs(node->vY) < 0.01f) node->vY = 0;

         node->pX += node->vX * fDeltaTime;
         node->pY += node->vY * fDeltaTime;
      }
   }

   void SelfCollisionSim(Mesh* mesh)
   {
      for (auto& node1 : mesh->Nodes)
      {
         for (auto& node2 : mesh->Nodes)
         {
            if (node1 != node2)
            {
               if (CircleCollisionDetect(node1, node2))
               {
                  float distance = sqrtf((node1->pX - node2->pX) * (node1->pX - node2->pX) +
                     (node1->pY - node2->pY) * (node1->pY - node2->pY));
                  float overlap = distance - node1->radius - node2->radius;

                  // resolving static collision
                  node1->pX -= overlap * 0.5f * (node1->pX - node2->pX) / distance;
                  node1->pY -= overlap * 0.5f * (node1->pY - node2->pY) / distance;

                  node2->pX += overlap * 0.5f * (node1->pX - node2->pX) / distance;
                  node2->pY += overlap * 0.5f * (node1->pY - node2->pY) / distance;
               }
            }
         }
      }
   }

   bool CircleSelected(float x1, float y1, float radius)
   {
      return ((m_MousePosition.x - offset.x - x1) * (m_MousePosition.x - offset.x - x1) + (m_MousePosition.y - offset.y - y1) * (m_MousePosition.y - offset.y - y1) < (radius * radius) * radiusSelectFactor);
   }

   Node* SelectNode(Mesh* mesh)
   {
      for (auto& node : mesh->Nodes)
      {
         if (CircleSelected(node->pX, node->pY, node->radius))
            return node;
      }
      return nullptr;
   }

   LineSegment* SelectLinePoint()
   {
      for (auto& line : lineSegments)
      {
         if (CircleSelected(line->sx, line->sy, globalLineRadius))
         {
            selectedLineNode = true;
            return line;
         }
         else if (CircleSelected(line->ex, line->ey, globalLineRadius))
         {
            selectedLineNode = false;
            return line;
         }
      }
      return nullptr;
   }


   void DeleteMesh(Mesh* mesh)
   {
      for (auto& spring : mesh->Springs)
         delete spring;
      for (auto& node : mesh->Nodes)
         delete node;
      mesh->Nodes.clear();
      mesh->Springs.clear();
      delete mesh;
   }

   bool CircleCollisionDetect(Node* n1, Node* n2)
   {
      return ((n1->radius + n2->radius) * (n1->radius + n2->radius) >= (n1->pX - n2->pX) * (n1->pX - n2->pX) + (n1->pY - n2->pY) * (n1->pY - n2->pY));
   }

   void AddLine()
   {
      lineSegments.push_back(new LineSegment{ m_MousePosition.x - offset.x, m_MousePosition.y - offset.y, m_MousePosition.x - offset.x + m_ScreenWidth/10.0f, m_MousePosition.y - offset.y + m_ScreenHeight/10.0f, globalLineRadius});
   }

   // returns the world coordinates of a given screen coordinate input
   void ScreenToWorld(float& X, float& Y)
   {
      X = (X - m_ScreenWidth / 2.0f) / scale + offset.x;
      Y = (Y - m_ScreenHeight / 2.0f) / scale + offset.y;
   }

   void HandleCamera()
   {
      auto io = ImGui::GetIO();

      /*   START USER INPUT   */
      if (!io.WantCaptureMouse)
      {
         // start mouse panning
         if (m_Mouse[BUTTON_1].bPressed)
         {
            mousePan = { (m_MousePosition.x - prevOffset.x * scale), (m_MousePosition.y - prevOffset.y * scale) };
         }
         // update offset while dragging
         if (m_Mouse[BUTTON_1].bHeld)
         {
            offset = { (m_MousePosition.x - mousePan.x) / (scale), (m_MousePosition.y - mousePan.y) / (scale) };
         }
         // drag end, store previous offset
         else if (m_Mouse[BUTTON_1].bReleased)
         {
            prevOffset = offset;
         }
         // zoom in or out focused on the mouse location
         if (m_MouseScroll.up || m_MouseScroll.down)
         {
            float mouseXBefore = m_MousePosition.x;
            float mouseYBefore = m_MousePosition.y;

            ScreenToWorld(mouseXBefore, mouseYBefore);

            float mouseXAfter = m_MousePosition.x;
            float mouseYAfter = m_MousePosition.y;

            ScreenToWorld(mouseXAfter, mouseYAfter);

            offset.x -= (mouseXBefore - mouseXAfter);
            offset.y -= (mouseYBefore - mouseYAfter);

            prevOffset = offset;
         }

         // reset view
         if (m_Keys[KEY_R].bPressed)
         {
            scale = 0.38;
            prevOffset = offset;
         }
      }
   }

   void GetInput()
   {
      if (m_Mouse[BUTTON_1].bPressed)
      {
         selectedNode = SelectNode(testMesh);
         selectedLine = SelectLinePoint();
      }
      if (selectedNode != nullptr)
      {
         if (m_Mouse[BUTTON_1].bHeld)
         {
            selectedNode->pX = m_MousePosition.x - offset.x;
            selectedNode->pY = m_MousePosition.y - offset.y;
         }
         else if (m_Mouse[BUTTON_2].bReleased)
            selectedNode = nullptr;
      }
      else if (selectedLine != nullptr)
      {
         if (m_Mouse[BUTTON_1].bHeld && selectedLineNode)
         {
            selectedLine->sx = m_MousePosition.x - offset.x;
            selectedLine->sy = m_MousePosition.y - offset.y;
         }
         else if (m_Mouse[BUTTON_1].bHeld)
         {
            selectedLine->ex = m_MousePosition.x - offset.x;
            selectedLine->ey = m_MousePosition.y - offset.y;
         }
         else if (m_Mouse[BUTTON_2].bReleased)
            selectedNode = nullptr;
      }
      else
      {
         HandleCamera();

         mat4 view = mat4::perspective(scale, 1.0f, 1.0f, 0.0f);
         mat4 translate = mat4::translation(Vec3(offset.x * 2.0f / m_ScreenWidth, offset.y * 2.0f / m_ScreenHeight, 0.0f));

         // m_Shader->setUniformMat4("vw_matrix", view);
         m_Shader->setUniformMat4("ml_matrix", translate);
      }

      //if (m_MouseScroll.up)
      //{
      //   scale *= 1.1;
      //}
      //else if (m_MouseScroll.down)
      //{
      //   scale *= 0.9;
      //}

      if (m_Keys[KEY_L].bPressed)
         AddLine();

      if (m_Keys[KEY_S].bHeld)
      {
         testMesh->Nodes[0]->pX -= 10.0f;
         testMesh->Nodes[0]->pY -= 10.0f;
      }

      if (m_Keys[KEY_W].bHeld)
      {
         testMesh->Nodes[0]->pX += 10.0f;
         testMesh->Nodes[0]->pY += 10.0f;
      }

      if (m_Keys[KEY_R].bPressed)
      {
         DeleteMesh(testMesh);
         CREATE_MESH;
      }

      if (m_Keys[KEY_F2].bPressed)
      {
         SetFullScreen();
      }
   }

   void RenderControlMenu()
   {
      ImGui::Begin("Debug");
      ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
#if DEBUG_MODE
      ImGui::Text("Mouse: %.3f %.3f", m_MousePosition.x - offset.x, m_MousePosition.y - offset.y);
      ImGui::Text("Node0: %.3f %.3f", testMesh->Nodes[0]->pX, testMesh->Nodes[0]->pY);
      ImGui::Text("N fxy: %.3f %.3f", testMesh->Nodes[0]->fX, testMesh->Nodes[0]->fY);
#endif
      ImGui::SliderInt("# Iterations", &numIterations, 1, 20);
      ImGui::SliderFloat("Dampening", &globalDampingFactor, 0.1f, 5.0f);
      ImGui::SliderFloat("Stiffness", &globalSpringStiffness, 0.1f, 2000.0f);
      ImGui::SliderFloat("Node Radius", &globalNodeRadius, 0.1f, 100.0f);
      ImGui::SliderFloat("Line Radius", &globalLineRadius, 0.1f, 100.0f);
      ImGui::SliderFloat("Dynamic Coeff", &dynamicSpringFactor, 1.0f, 2.0f);
      ImGui::SliderFloat("Gravity", &gravity, 0.0f, 1000.0f);
      ImGui::Checkbox("Normal Force", &normalForce);
      ImGui::SetWindowFontScale(1.5f);
      ImGui::End();
   }

   /*--- Simulation Variables ---*/
   float scale = 1.0f;
   float mZoom;
   float mRatio;
   Vec2 mousePan, offset, prevOffset = { 0.0f, 0.0f };

   Mesh* testMesh;
   float fov, aspect, nearF, farF;
   float ratio;

   std::chrono::system_clock::time_point prevTime = chrono::system_clock::now();
   std::chrono::system_clock::time_point currTime = chrono::system_clock::now();
   float elapsedTime;
   bool run = false;

   Node* selectedNode = nullptr;
   LineSegment* selectedLine = nullptr;
   bool selectedLineNode = false;

   vector<Node*> virtualNodes;
   vector<LineSegment*> lineSegments;

   bool normalForce = true;
   /*--- End Simulation Variables ---*/

   bool OnUserCreate()
   {
      // testMesh = CreateMesh(m_ScreenWidth * 4 / 10, m_ScreenHeight * 2 / 10, m_ScreenWidth / 2, m_ScreenHeight / 2, 8, 5);
      CREATE_MESH;

      scale = 1;
      ratio = m_ScreenWidth / m_ScreenHeight;

      numIterations = 4;
      return true;
   }

   bool OnUserUpdate()
   {
      currTime = chrono::system_clock::now();
      chrono::duration<float> elapsedTimeCalc = currTime - prevTime;
      elapsedTime = elapsedTimeCalc.count();
      prevTime = currTime;

      auto io = ImGui::GetIO();

      NormalizeScreen();

      /*--- Begin Physics Simulation ---*/
#if 0
      if (!run)
      {
         for (int i = 4; i < testMesh->Springs.size() - 4; i++)
         {
            delete testMesh->Springs[i];
         }

         for (int i = 4; i < testMesh->Springs.size() - 4; i++)
         {
            testMesh->Springs.erase(testMesh->Springs.begin() + i);
         }
         run = !run;
         testMesh->height = 2;
         testMesh->width = 2;

         testMesh->Springs.resize(4);
      }
#endif
      elapsedTime /= numIterations;

      for (int i = 0; i < numIterations; i++)
      {
         CalculateSpringForces(testMesh);
         CalculateGravitationalForces(testMesh);
         CalculateLineNormal(testMesh);
         CalculateNodeVelocity(testMesh, elapsedTime);
         SelfCollisionSim(testMesh);
      }

      if (!io.WantCaptureMouse)
      {
         GetInput();
      }

      RenderMesh(testMesh);
      for (auto& line : lineSegments)
      {
         RenderLine(line->sx, line->sy, line->ex, line->ey, 99999999, globalLineRadius);
         RenderQuad(line->sx, line->sy, globalLineRadius, globalLineRadius, 0.0f, &circleTexture);
         RenderQuad(line->ex, line->ey, globalLineRadius, globalLineRadius, 0.0f, &circleTexture);
      }

      for (auto& virtualNode : virtualNodes)
      {
         delete virtualNode;
      }
      virtualNodes.clear();

      RenderControlMenu();

      if (m_Keys[KEY_ESCAPE].bPressed)
         return false;

      return true;
   }

};

int main()
{
   SoftbodySim game;

   game.Construct("Softbody Simulation", 1600, 1200);

   game.Start();
}