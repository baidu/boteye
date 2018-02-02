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

#ifndef __AIP_FACE_H__
#define __AIP_FACE_H__

#include "base/base.h"

namespace aip {

    class Face: public AipBase
    {
    public:
        
        std::string _detect =
            "https://aip.baidubce.com/rest/2.0/face/v1/detect";
        
        std::string _match =
            "https://aip.baidubce.com/rest/2.0/face/v2/match";
        
        std::string _identify =
            "https://aip.baidubce.com/rest/2.0/face/v2/identify";
        
        std::string _verify =
            "https://aip.baidubce.com/rest/2.0/face/v2/verify";
        
        std::string _user_add =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/user/add";
        
        std::string _user_update =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/user/update";
        
        std::string _user_delete =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/user/delete";
        
        std::string _user_get =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/user/get";
        
        std::string _group_getlist =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/group/getlist";
        
        std::string _group_getusers =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/group/getusers";
        
        std::string _group_adduser =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/group/adduser";
        
        std::string _group_deleteuser =
            "https://aip.baidubce.com/rest/2.0/face/v2/faceset/group/deleteuser";
        

        Face(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }

        std::string vector_join_base64(const std::vector<std::string> & v_images) {
            std::string images;
            size_t count = v_images.size();
            for (size_t i = 0; i < count;i++)
            {
                std::string image = v_images[i];
                images += base64_encode(image.c_str(), (int) image.size());
                if (i != count) {
                    images += ",";
                }

            }
            return images;
        }
        
        /**
         * detect
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * max_face_num 最多处理人脸数目，默认值1
         * face_fields 包括age,beauty,expression,faceshape,gender,glasses,landmark,race,qualities信息，逗号分隔，默认只返回人脸框、概率和旋转角度
         */
        Json::Value detect(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_detect, null, data, null);

            return result;
        }
        
        /**
         * match
         * @param images vector多图图像文件二进制内容，vector中每一项可以使用aip::get_file_content函数获取
         * options 可选参数:
         * ext_fields 返回质量信息，取值固定:目前支持qualities(质量检测)。(对所有图片都会做改处理)
         * image_liveness 返回的活体信息，“faceliveness,faceliveness” 表示对比对的两张图片都做活体检测；“,faceliveness” 表示对第一张图片不做活体检测、第二张图做活体检测；“faceliveness,” 表示对第一张图片做活体检测、第二张图不做活体检测
         */
        Json::Value match(
            const std::vector<std::string> & images,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["images"] = vector_join_base64(images);

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_match, null, data, null);

            return result;
        }
        
        /**
         * identify
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B，多个用户组id，用逗号分隔
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * ext_fields 特殊返回信息，多个用逗号分隔，取值固定:目前支持 faceliveness(活体检测)
         * user_top_num 返回用户top数，默认为1，最多返回5个
         */
        Json::Value identify(
            std::string const & group_id,
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["group_id"] = group_id;
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_identify, null, data, null);

            return result;
        }
        
        /**
         * verify
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B，多个用户组id，用逗号分隔
         * options 可选参数:
         * top_num 返回用户top数，默认为1
         * ext_fields 特殊返回信息，多个用逗号分隔，取值固定:目前支持 faceliveness(活体检测)，活体检测参考分数0.4494，以上则可认为是活体（测试期间）
         */
        Json::Value verify(
            std::string const & uid,
            std::string const & image,
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["uid"] = uid;
            data["image"] = base64_encode(image.c_str(), (int) image.size());
            data["group_id"] = group_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_verify, null, data, null);

            return result;
        }
        
        /**
         * user_add
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * @param user_info 用户资料，长度限制256B
         * @param group_id 用户组id，标识一组用户（由数字、字母、下划线组成），长度限制128B。如果需要将一个uid注册到多个uid下，group_id需要用多个逗号分隔，每个group_id长度限制为48个英文字符
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * action_type 参数包含append、replace。如果为“replace”，则每次注册时进行替换replace（新增或更新）操作，默认为append操作
         */
        Json::Value user_add(
            std::string const & uid,
            std::string const & user_info,
            std::string const & group_id,
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["uid"] = uid;
            data["user_info"] = user_info;
            data["group_id"] = group_id;
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_user_add, null, data, null);

            return result;
        }
        
        /**
         * user_update
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * @param user_info 用户资料，长度限制256B
         * @param group_id 更新指定groupid下uid对应的信息
         * options 可选参数:
         * action_type 目前仅支持replace，uid不存在时，不报错，会自动变为注册操作；未选择该参数时，如果uid不存在会提示错误
         */
        Json::Value user_update(
            std::string const & uid,
            std::string const & image,
            std::string const & user_info,
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["uid"] = uid;
            data["image"] = base64_encode(image.c_str(), (int) image.size());
            data["user_info"] = user_info;
            data["group_id"] = group_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_user_update, null, data, null);

            return result;
        }
        
        /**
         * user_delete
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * @param group_id 删除指定groupid下uid对应的信息
         * options 可选参数:
         */
        Json::Value user_delete(
            std::string const & uid,
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["uid"] = uid;
            data["group_id"] = group_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_user_delete, null, data, null);

            return result;
        }
        
        /**
         * user_get
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * group_id 选择指定group_id则只查找group列表下的uid内容，如果不指定则查找所有group下对应uid的信息
         */
        Json::Value user_get(
            std::string const & uid,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["uid"] = uid;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_user_get, null, data, null);

            return result;
        }
        
        /**
         * group_getlist
         * options 可选参数:
         * start 默认值0，起始序号
         * end 返回数量，默认值100，最大值1000
         */
        Json::Value group_getlist(
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_group_getlist, null, data, null);

            return result;
        }
        
        /**
         * group_getusers
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         * start 默认值0，起始序号
         * end 返回数量，默认值100，最大值1000
         */
        Json::Value group_getusers(
            std::string const & group_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["group_id"] = group_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_group_getusers, null, data, null);

            return result;
        }
        
        /**
         * group_adduser
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B，多个用户组id，用逗号分隔
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * @param src_group_id 从指定group里复制信息
         * options 可选参数:
         */
        Json::Value group_adduser(
            std::string const & group_id,
            std::string const & uid,
            std::string const & src_group_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["group_id"] = group_id;
            data["uid"] = uid;
            data["src_group_id"] = src_group_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_group_adduser, null, data, null);

            return result;
        }
        
        /**
         * group_deleteuser
         * @param group_id 用户组id（由数字、字母、下划线组成），长度限制128B，多个用户组id，用逗号分隔
         * @param uid 用户id（由数字、字母、下划线组成），长度限制128B
         * options 可选参数:
         */
        Json::Value group_deleteuser(
            std::string const & group_id,
            std::string const & uid,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["group_id"] = group_id;
            data["uid"] = uid;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_group_deleteuser, null, data, null);

            return result;
        }
        
    };
}
#endif
