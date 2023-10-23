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
      double m_auv_lat;
      //! AUV longitude.
      double m_auv_lon;
      //! True if points_to_visit are given as pairs
      bool m_PTV_ok;
      //! True if plan already calculated and sent
      bool m_plan_sent;
      //! Plan specification of plan to run
      IMC::PlanSpecification m_plan_to_run;
      //! Generator for request id
      Math::Random::Generator* m_gen;

      //! Constructor.
      //! @param[in] name task name.
      //! @param[in] ctx context.
      Task(const std::string& name, Tasks::Context& ctx):
        DUNE::Tasks::Task(name, ctx),
        m_vstate(IMC::VehicleState::VS_BOOT),
        m_plan_sent(false),
        m_PTV_ok(false),
        m_in_mission(false)
      {
        // Parameter handling 
        paramActive(Tasks::Parameter::SCOPE_GLOBAL,
                    Tasks::Parameter::VISIBILITY_USER);

        param("Points to Visit", m_args.points_to_visit)
        .defaultValue("")
        .description("Points we want to visit with shortes possible non-crossing path.");

        // Init of generator
        m_gen = Math::Random::Factory::create(Math::Random::Factory::c_default);

        // Init of plan specification
        m_plan_to_run.plan_id = "PlanVisit";
        m_plan_to_run.description = "Visiting given points in ini file in optimal order based on range";

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

        if (m_in_mission & (msg->last_outcome == PlanControlState::LPO_SUCCESS))
        {
          requestDeactivation();
        }
      }

      void
      consume(const IMC::EstimatedState* msg)
      {
        setEntityState(IMC::EntityState::ESTA_NORMAL, Status::CODE_ACTIVE);
        Coordinates::toWGS84(*msg, m_auv_lat, m_auv_lon);
      }

      //! Calculate weight matrix for PTV 
      //! A weight is defined as the range between two points
      //! @param[out] the calculated weight matrix
      std::vector<std::vector<double>>
      calculateWeights()
      {
        // Initialize weight matrix
        unsigned int n_points = m_args.points_to_visit.size()/2+1;
        std::vector<std::vector<double>> weights(n_points, std::vector<double>(n_points,0));

        // Loop through all points, 
        double b, r;
        for(unsigned int i = 0; i < m_args.points_to_visit.size(); i = i + 2)
        {
          // Calculate range from point to current auv pos
          Coordinates::WGS84::getNEBearingAndRange(m_auv_lat, m_auv_lon, m_args.points_to_visit[i], m_args.points_to_visit[i+1], &b, &r);
          weights[0][i/2+1] = r; 
          weights[i/2+1][0] = r;
          for(unsigned int j = i; j < m_args.points_to_visit.size(); j = j + 2)
          {
            if (i==j) 
            {
              // Range between each themselves are zero
              weights[i/2+1][j/2+1] = 0;
            }
            else 
            {
              // Caclculate distance between different points
              Coordinates::WGS84::getNEBearingAndRange(m_args.points_to_visit[i], m_args.points_to_visit[i+1], m_args.points_to_visit[j], m_args.points_to_visit[j+1], &b, &r);
              weights[i/2+1][j/2+1] = r;
              weights[j/2+1][i/2+1] = r;
            }
          } 
        }
        // Return
        return weights;
      }

      //! Brute force traveling salesman algorithm. Based on https://www.geeksforgeeks.org/traveling-salesman-problem-tsp-implementation/
      //! @param[in] the weight matrix
      //! @param[out] the ordering of the indices
      std::vector<unsigned int> 
      TSP(std::vector<std::vector<double>>& weights)
      {
        // Store all indices of vertex apart from source vertex
        std::vector<unsigned int> indices;
        std::vector<unsigned int> best_indices; 
        for (unsigned int i = 0; i < m_args.points_to_visit.size(); i = i+2) 
        {
          indices.push_back(i/2+1);
        }

        // Store minimum weight Hamiltonian Cycle
        double min_path = DBL_MAX;

        // Loop through all possible combinations to find shortes route. 
        do 
        {
          double current_pathweight = 0;

          int k = 0;
          for (unsigned int i = 0; i < indices.size(); i++)
          {
            current_pathweight += weights[k][indices[i]];
            k = indices[i];
          }
          current_pathweight += weights[k][0];

          if (current_pathweight < min_path)
          {
            min_path = current_pathweight;
            best_indices = indices;  
          }
        }
        while
        (
          next_permutation(indices.begin(), indices.end())
        );

        // Subtract 1 from each vertex to get back original index
        std::transform(best_indices.begin(), best_indices.end(), best_indices.begin(), [](int element) {
            return element - 1;
        });

        // Return
        return best_indices;
      }

      //! Creates the plan based on the ordering of the indices
      //! Stores plan in m_plan_to_send
      //! @param[in] the ordering of the indices
      void
      createPlan(std::vector<unsigned int>& indices) 
      {
        // Code adapted from assignment paper
        for(unsigned int i = 0; i < indices.size(); i++)
        {
          double lat = m_args.points_to_visit[indices[i]*2];
          double lon = m_args.points_to_visit[indices[i]*2 + 1];
          IMC::Goto goto_maneuver;
          goto_maneuver.lat = lat;
          goto_maneuver.lon = lon;
          goto_maneuver.speed = 1.6;
          goto_maneuver.speed_units = IMC::SpeedUnits::SUNITS_METERS_PS;
          goto_maneuver.z = 0;
          goto_maneuver.z_units = IMC::Z_DEPTH;
          // Set Plan Maneuver
          IMC::PlanManeuver pman;
          std::stringstream man_name;
          man_name << "Goto" << (i);
          pman.maneuver_id = man_name.str();
          pman.data.set(goto_maneuver);
          m_plan_to_run.maneuvers.push_back(pman);

          if (i == 0)
          {
            m_plan_to_run.start_man_id = pman.maneuver_id;
          }
          else
          {
            // Set Plan Transition
            IMC::PlanTransition ptrans;
            std::stringstream prev_man_name;
            prev_man_name << "Goto" << (i) - 1;
            ptrans.source_man = prev_man_name.str();
            ptrans.dest_man = man_name.str();
            ptrans.conditions = "ManeuverIsDone";
            m_plan_to_run.transitions.push_back(ptrans);
          }
        }

        // Add last goto maneuver back to current location
        IMC::Goto goto_maneuver;
        goto_maneuver.lat = m_auv_lat;
        goto_maneuver.lon = m_auv_lon;
        goto_maneuver.speed = 1.6;
        goto_maneuver.speed_units = IMC::SpeedUnits::SUNITS_METERS_PS;
        goto_maneuver.z = 0;
        goto_maneuver.z_units = IMC::Z_DEPTH;
        // Set Plan Maneuver
        IMC::PlanManeuver pman;
        std::stringstream man_name;
        man_name << "Goto" << (indices.size());
        pman.maneuver_id = man_name.str();
        pman.data.set(goto_maneuver);
        m_plan_to_run.maneuvers.push_back(pman);

        // Set Plan Transition
        IMC::PlanTransition ptrans;
        std::stringstream prev_man_name;
        prev_man_name << "Goto" << indices.size() - 1;
        ptrans.source_man = prev_man_name.str();
        ptrans.dest_man = man_name.str();
        ptrans.conditions = "ManeuverIsDone";
        m_plan_to_run.transitions.push_back(ptrans);
      }

      //! Sends the plan using PlanControl message
      void
      sendPlan()
      {
        IMC::PlanControl plan_control;
        plan_control.type = IMC::PlanControl::PC_REQUEST;
        plan_control.op = IMC::PlanControl::PC_START;
        plan_control.request_id = m_gen->random() & 0xFFFF;
        plan_control.plan_id = m_plan_to_run.plan_id;
        plan_control.arg.set(m_plan_to_run);
        plan_control.setDestination(getSystemId());
        dispatch(plan_control);
      }

      //! Main loop.
      void
      onMain(void)
      {
        while (!stopping())
        {
          if (isActive() & !m_plan_sent & (m_vstate == IMC::VehicleState::VS_SERVICE))
          {
            // Generate plan
            // Need to calculate the distances between the points. We store them in a weight matrice. 
            std::vector<std::vector<double>> weights = calculateWeights();
            // Find the order of visit by calling the TSP algorithm
            std::vector<unsigned int> index = TSP(weights);
            // Create plan message stored in m_plan_to_run
            createPlan(index);
            // Send plan stored in m_plan_to_run
            sendPlan();
            m_plan_sent = true;
          }
          waitForMessages(1.0);
        }
      }
    };
  }
}

DUNE_TASK
