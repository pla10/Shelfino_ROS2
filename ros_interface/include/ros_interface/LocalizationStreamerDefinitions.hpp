
#ifndef LOCALIZATION_STREAMER_DEFINITIONS_HPP__
#define LOCALIZATION_STREAMER_DEFINITIONS_HPP__

#include <iostream>
#include <string>
#include <mutex>
#include <array>
#include <sstream>

#include <Eigen/Dense>
#include <localizationMatrix.h>
#include <json.hpp>

namespace Localization {
    constexpr char TOPIC[] = "LOC";

    struct data_t {

        //const std::string TOPIC = "LOC";

        int time_ns;  ///< The timestamp of the data.

        localizationMatrix::StateVec_t state;

        localizationMatrix::StateCovMatrix_t covariance;

        double &x, &y, &theta, &v, &omega;  ///< alias to state array.


        /// @brief Constructor.
        ///
        /// This is the constructor of the structure.
        data_t():time_ns(0), state(5), covariance(5,5), x(state(0)),y(state(1)), theta(state(2)), v(state(3)), omega(state(4)) { }

        /// @brief Constructor.
        ///
        /// This is the copy constructor of the structure.
        data_t(const data_t &in) :time_ns(in.time_ns), state(5), covariance(5,5), x(state(0)),y(state(1)), theta(state(2)), v(state(3)), omega(state(4)) {
            state = in.state;
            covariance = in.covariance;
        }


        void to_json(nlohmann::json &j){

            nlohmann::json j_state;
            j["time"]           = time_ns;
            j_state["x"]        = state[0];
            j_state["y"]        = state[1];
            j_state["theta"]    = state[2];
            j_state["v"]        = state[3];
            j_state["omega"]    = state[4];
            j["state"]          = j_state;

            nlohmann::json j_covariance = nlohmann::json::array();
            int count = 0;
            for(int i=0; i<localizationMatrix::STATE_SIZE; i++){
                for(int j=0; j<localizationMatrix::STATE_SIZE; j++, count++){
                    j_covariance[count] = covariance(j,i);
                }
            }

            j["covariance"]     = j_covariance;


        }

        /// @brief push the data in a string that will be published.
        ///
        /// Convert the data in a publishable format.
        std::string pushData() const {
            const char* SEP   = " ";         ///< The separator for the string.
            std::stringstream ss;            ///< The stream of the data information.
            ss.str("");
            ss.precision(17);
            ss << SEP << time_ns;

            /*
            for(uint8_t i=0;i<localizationMatrix::STATE_SIZE;i++){
                ss << SEP << state(i,0);
            }
            */
            for(uint8_t i=0;i<3;i++){
                ss << SEP << state(i,0);
            }
            ss << SEP << state(3,0)*std::cos(state(2,0));
            ss << SEP << state(3,0)*std::sin(state(2,0));
            ss << SEP << state(4,0); // omega
            ss << SEP << state(3,0); // v


            // Covariance matrix is symmetric so send only half of the matrix
            for(size_t i=0; i<localizationMatrix::STATE_SIZE;i++){
                for(size_t j=0; j<i+1;j++) {
                    ss << SEP << covariance(j,i);
                }
            }

            return ss.str();
        }

        /// @brief pull the published data in data_t.
        ///
        /// Convert the published data
        /// @return success of the operation
        bool pullData(std::string str)  {

            std::stringstream ss;
            ss.str(str);

            // TODO: add try catch to manage possible parsing error
            ss >> time_ns;
            ss >> time_ns;
            /*
            for(uint8_t i=0;i<localizationMatrix::STATE_SIZE;i++){
                ss >> state(0,i);
            }
             */
            for(uint8_t i=0;i<3;i++){
                ss >> state(i,0);
            }

            double v_tmp;
            ss >> v_tmp;
            ss >> v_tmp;
            ss >> state(4,0);
            ss >> state(3,0);



            // Covariance matrix is symmetric so send only half of the matrix
            for(size_t i=0; i<localizationMatrix::STATE_SIZE;i++){
                for(size_t j=0; j<i+1;j++) {
                    ss >> covariance(j,i);
                    covariance(i,j) = covariance(j,i);
                }
            }

            return true;
        }

    };



}

#endif
