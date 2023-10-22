//***************************************************************************
// Copyright 2007-2023 Universidade do Porto - Faculdade de Engenharia      *
// Laboratório de Sistemas e Tecnologia Subaquática (LSTS)                  *
//***************************************************************************
// This file is part of DUNE: Unified Navigation Environment.               *
//                                                                          *
// Commercial Licence Usage                                                 *
// Licencees holding valid commercial DUNE licences may use this file in    *
// accordance with the commercial licence agreement provided with the       *
// Software or, alternatively, in accordance with the terms contained in a  *
// written agreement between you and Faculdade de Engenharia da             *
// Universidade do Porto. For licensing terms, conditions, and further      *
// information contact lsts@fe.up.pt.                                       *
//                                                                          *
// Modified European Union Public Licence - EUPL v.1.1 Usage                *
// Alternatively, this file may be used under the terms of the Modified     *
// EUPL, Version 1.1 only (the "Licence"), appearing in the file LICENCE.md *
// included in the packaging of this file. You may not use this work        *
// except in compliance with the Licence. Unless required by applicable     *
// law or agreed to in writing, software distributed under the Licence is   *
// distributed on an "AS IS" basis, WITHOUT WARRANTIES OR CONDITIONS OF     *
// ANY KIND, either express or implied. See the Licence for the specific    *
// language governing permissions and limitations at                        *
// https://github.com/LSTS/dune/blob/master/LICENCE.md and                  *
// http://ec.europa.eu/idabc/eupl.html.                                     *
//***************************************************************************
// Author: F Maurer                                                         *
//***************************************************************************

// DUNE headers.
#include <DUNE/DUNE.hpp>

namespace Tutorial
{
  //! Insert short task description here.
  //!
  //! Insert explanation on task behaviour here.
  //! @author F Maurer
  namespace PlanVisit
  {
    using DUNE_NAMESPACES;

    struct Arguments 
    {
      std::vector<double> points_to_visit;
    };

    struct Task: public DUNE::Tasks::Task
    {
      Arguments m_args; 

      //! Vehicle State
      uint8_t m_vstate;
      //! True if executing plan.
      bool m_in_mission;
      //! Executing plan's progress.
      float m_progress;
      //! AUV latitude.
      double m_lat;
      //! AUV longitude.
      double m_lon;
      //! True if points_to_visit are given as pairs
      bool m_PTV_ok;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx)
      {
        // Parameter handling 
        paramActive(Tasks::Parameter::SCOPE_GLOBAL,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Points to Visit", m_args.points_to_visit)
        .defaultValue("")
        .description("Points we want to visit with shortes possible non-crossing path.");

        // Subscribe to messages 
        bind<IMC::EstimatedState>(this);
        bind<IMC::VehicleState>(this);
        bind<PlanControlState>(this);
      }

      //! Update internal state with new parameter values.
      void
      onUpdateParameters(void)
      {
        if (m_args.points_to_visit.size() % 2 != 0)
        {
          war("Odd number of points to visit input. Task is deactivated.");
          m_PTV_ok = false; 
        }
        else 
        {
          // Convert points to radians
          for (size_t i = 0; i < m_args.points_to_visit.size(); ++i) {
              m_args.points_to_visit[i] = Angles::radians(m_args.points_to_visit[i]);
          }
          // Set the PTV status to ok
          m_PTV_ok = true; 
        }
      }

      //! Reserve entity identifiers.
      void
      onEntityReservation(void)
      {
      }

      //! Resolve entity names.
      void
      onEntityResolution(void)
      {
      }

      //! Acquire resources.
      void
      onResourceAcquisition(void)
      {
      }

      //! Initialize resources.
      void
      onResourceInitialization(void)
      {
      }

      //! Release resources.
      void
      onResourceRelease(void)
      {
      }

      //! When activated
      void 
      onActivation(void)
      {
        if (m_PTV_ok)
        {
          setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        }
        else 
        {
          war("Cannot activate task since the given points to visit are not ok.");
        }
      }

      //! When deactivated
      void 
      onDeactivation(void)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_IDLE);
      }

      void
      consume(const IMC::VehicleState* msg)
      {
        m_vstate = msg->op_mode;
      }

      void
      consume(const IMC::PlanControlState* msg)
      {
        m_in_mission = msg->state == IMC::PlanControlState::PCS_EXECUTING;
        m_progress = msg->plan_progress;
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        Coordinates::toWGS84(*msg, m_lat, m_lon);
      }

      //! Calculate weight matrix for PTV
      void
      calculateWeights(std::vector<std::vector<double>>& weights)
      {
        double b, r;
        for(unsigned int i = 0; i < m_args.points_to_visit.size(); i = i + 2)
        {
          Coordinates::WGS84::getNEBearingAndRange(m_lat, m_lon, m_args.points_to_visit[i], m_args.points_to_visit[i+1], &b, &r);
          weights[0][i/2] = r; 
          weights[i/2][0] = r;
          for(unsigned int j = i; j < m_args.points_to_visit.size(); j = j + 2)
          {
            if (i==j) 
            {
              weights[i/2][j/2] = 0;
            }
            else 
            {
              Coordinates::WGS84::getNEBearingAndRange(m_args.points_to_visit[i], m_args.points_to_visit[i+1], m_args.points_to_visit[j], m_args.points_to_visit[j+1], &b, &r);
              weights[i/2+1][j/2] = r;
              weights[j/2][i/2+1] = r;
            }
          } 
        }
      }

      //! Brute force traveling salesman algorithm. Based on https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/
      std::vector<unsigned int> 
      TSP(std::vector<std::vector<double>>& weights)
      {
        // Store all vertex apart from source vertex
        std::vector<unsigned int> vertex;
        std::vector<unsigned int> best_vertex; 
        for (unsigned int i = 0; i < m_args.points_to_visit.size(); i++) 
        {
          vertex.push_back(i+1);
        }

        // Store minimum weight Hamiltonian Cycle
        double min_path = DBL_MAX;

        do 
        {
          double current_pathweight = 0;

          int k = 0;
          for (unsigned int i = 0; i < vertex.size(); i++)
          {
            current_pathweight += weights[k][vertex[i]];
            k = vertex[i];
          }
          current_pathweight += weights[k][0];

          if (current_pathweight < min_path)
          {
            min_path = current_pathweight;
            best_vertex = vertex; 
          }
        }
        while
        (
          next_permutation(vertex.begin(), vertex.end())
        );
        return best_vertex;
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
