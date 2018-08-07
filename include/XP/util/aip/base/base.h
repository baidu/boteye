/**
 * Copyright (c) 2017 Baidu.com, Inc. All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
 * the License. You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
 * an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
 * specific language governing permissions and limitations under the License.
 *
 * @author baidu aip
 */
#ifndef __AIP_BASE_H__
#define __AIP_BASE_H__

#define __AIP_SDK_VERSION__ "0.3.0"

#include <memory>
#include <cstring>
#include "http.h"
#include <json/json.h>
#include "base64.h"
#include "curl/curl.h"
#include "utils.h"

namespace aip {
    
    static const char* CURL_ERROR_CODE = "curl_error_code";
    static const std::string ACCESS_TOKEN_URL = "https://aip.baidubce.com/oauth/2.0/token";
    static const std::map<std::string, std::string> null;
    
    class AipBase
    {
    private:
        std::string _app_id;
        int _expired_time;
        bool _is_bce;
        bool _has_decide_type;
        std::string _scope;
        
    protected:
        std::string getAccessToken()
        {
            time_t now = time(NULL);
            if (!access_token.empty())
            {
                return this->access_token;
            }
            
            if (now < this->_expired_time - 60 * 60 * 24)
            {
                return this->access_token;
            }
            
            std::string response;
            std::map<std::string, std::string> params;
            
            params["grant_type"] = "client_credentials";
            params["client_id"] = this->ak;
            params["client_secret"] = this->sk;
            int status_code = this->client.get(
                                               ACCESS_TOKEN_URL,
                                               &params,
                                               nullptr,
                                               &response
                                               );
            
            Json::Value obj;
            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj.toStyledString();
            }
            
            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            this->access_token = obj["access_token"].asString();
            this->_expired_time = obj["expires_in"].asInt() + (int) now;
            this->_scope = obj["scope"].asString();
            return this->access_token;
        }
        
    public:
        std::string ak;
        std::string sk;
        HttpClient client;
        Json::CharReaderBuilder crbuilder;
        std::string access_token;
        AipBase(const std::string & app_id, const std::string & ak, const std::string & sk):
        _app_id(app_id),
        _is_bce(false),
        _has_decide_type(false),
        ak(ak),
        sk(sk)
        {
            if (_app_id == "")
            {
            }
        }
        
        void setConnectionTimeoutInMillis(int connect_timeout)
        {
            this->client.setConnectTimeout(connect_timeout);
        }
        
        void setSocketTimeoutInMillis(int socket_timeout)
        {
            this->client.setSocketTimeout(socket_timeout);
        }
        
        void setDebug(bool debug)
        {
            this->client.setDebug(debug);
        }
        
        std::string getAk() {
            return ak;
        }
        
        Json::Value request(
                            std::string url,
                            std::map<std::string, std::string> const & params,
                            std::string const & data,
                            std::map<std::string, std::string> const & headers)
        {
            std::string response;
            Json::Value obj;
            std::string body;
            auto headers_for_sign = headers;
            
            auto temp_params = params;
            
            temp_params["charset"] = "UTF-8";
            
            this->prepare_request(url, temp_params, headers_for_sign);
            
            int status_code = this->client.post(url, &temp_params, data, &headers_for_sign, &response);
            
            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj;
            }
            
            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            
            return obj;
        }
        
        Json::Value request(
                            std::string url,
                            std::map<std::string, std::string> const & params,
                            std::map<std::string, std::string> const & data,
                            std::map<std::string, std::string> const & headers)
        {
            std::string response;
            Json::Value obj;
            
            auto headers_for_sign = headers;
            auto temp_params = params;
            
            this->prepare_request(url, temp_params, headers_for_sign);
            
            int status_code = this->client.post(url, &temp_params, data, &headers_for_sign, &response);
            
            if (status_code != CURLcode::CURLE_OK) {
                obj[CURL_ERROR_CODE] = status_code;
                return obj;
            }
            
            std::string error;
            std::unique_ptr<Json::CharReader> reader(crbuilder.newCharReader());
            reader->parse(response.data(), response.data() + response.size(), &obj, &error);
            
            return obj;
        }
        
        void prepare_request(std::string url,
                             std::map<std::string, std::string> & params,
                             std::map<std::string, std::string> & headers)
        {
            
            params["aipSdk"] = "C";
            params["aipSdkVersion"] = __AIP_SDK_VERSION__;
            params["channel"] = "roboticvision";
            
            if (_has_decide_type) {
                if (_is_bce) {
                    std::string method = "POST";
                    sign(method, url, params, headers, ak, sk);
                } else {
                    params["access_token"] = this->getAccessToken();
                }
                
                return;
            }
            
            if (getAccessToken() == "") {
                _is_bce = true;
                
            } else {
                
                const char * t = std::strstr(this->_scope.c_str(), "brain_all_scope");
                
                if (t == NULL)
                {
                    _is_bce = true;
                }
            }
            
            _has_decide_type = true;
            prepare_request(url, params, headers);
        }
        
        
    };
    
}
#endif
