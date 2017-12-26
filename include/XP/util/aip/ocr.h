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

#ifndef __AIP_OCR_H__
#define __AIP_OCR_H__

#include "base/base.h"

namespace aip {

    class Ocr: public AipBase
    {
    public:

    
        std::string _general_basic =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general_basic";
        
        std::string _accurate_basic =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate_basic";
        
        std::string _general =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general";
        
        std::string _accurate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/accurate";
        
        std::string _general_enhanced =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/general_enhanced";
        
        std::string _webimage =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/webimage";
        
        std::string _idcard =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/idcard";
        
        std::string _bankcard =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/bankcard";
        
        std::string _driving_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/driving_license";
        
        std::string _vehicle_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/vehicle_license";
        
        std::string _license_plate =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/license_plate";
        
        std::string _business_license =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/business_license";
        
        std::string _receipt =
            "https://aip.baidubce.com/rest/2.0/ocr/v1/receipt";
        
        std::string _table_recognize =
            "https://aip.baidubce.com/rest/2.0/solution/v1/form_ocr/request";
        
        std::string _table_result_get =
            "https://aip.baidubce.com/rest/2.0/solution/v1/form_ocr/get_request_result";
        

        Ocr(const std::string & app_id, const std::string & ak, const std::string & sk): AipBase(app_id, ak, sk)
        {
        }
        
        /**
         * general_basic
         * 用户向服务请求识别某张图中的所有文字
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_basic(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_basic, null, data, null);

            return result;
        }
        
        /**
         * general_basic_url
         * 用户向服务请求识别某张图中的所有文字
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效

         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_basic_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_basic, null, data, null);

            return result;
        }
        
        /**
         * accurate_basic
         * 用户向服务请求识别某张图中的所有文字，相对于通用文字识别该产品精度更高，但是没有免费额度，如果您需要使用该产品，您可以在产品页面点击合作咨询或加入文字识别的官网QQ群：631977213向管理员申请试用。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value accurate_basic(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_accurate_basic, null, data, null);

            return result;
        }
        
        /**
         * general
         * 用户向服务请求识别某张图中的所有文字，并返回文字在图中的位置信息。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general, null, data, null);

            return result;
        }
        
        /**
         * general_url
         * 用户向服务请求识别某张图中的所有文字，并返回文字在图中的位置信息。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效

         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general, null, data, null);

            return result;
        }
        
        /**
         * accurate
         * 用户向服务请求识别某张图中的所有文字，相对于通用文字识别（含位置信息版）该产品精度更高，但是没有免费额度，如果您需要使用该产品，您可以在产品页面点击合作咨询或加入文字识别的官网QQ群：631977213向管理员申请试用。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * vertexes_location 是否返回文字外接多边形顶点位置，不支持单字位置。默认为false
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value accurate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_accurate, null, data, null);

            return result;
        }
        
        /**
         * general_enhanced
         * 某些场景中，图片中的中文不光有常用字，还包含了生僻字，这时用户需要对该图进行文字识别，应使用通用文字识别（含生僻字版）。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_enhanced(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_enhanced, null, data, null);

            return result;
        }
        
        /**
         * general_enhanced_url
         * 某些场景中，图片中的中文不光有常用字，还包含了生僻字，这时用户需要对该图进行文字识别，应使用通用文字识别（含生僻字版）。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效

         * options 可选参数:
         * language_type 识别语言类型，默认为CHN_ENG。可选值包括：<br/>- CHN_ENG：中英文混合；<br/>- ENG：英文；<br/>- POR：葡萄牙语；<br/>- FRE：法语；<br/>- GER：德语；<br/>- ITA：意大利语；<br/>- SPA：西班牙语；<br/>- RUS：俄语；<br/>- JAP：日语；<br/>- KOR：韩语；
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         * probability 是否返回识别结果中每一行的置信度
         */
        Json::Value general_enhanced_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_general_enhanced, null, data, null);

            return result;
        }
        
        /**
         * webimage
         * 用户向服务请求识别一些网络上背景复杂，特殊字体的文字。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         */
        Json::Value webimage(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_webimage, null, data, null);

            return result;
        }
        
        /**
         * webimage_url
         * 用户向服务请求识别一些网络上背景复杂，特殊字体的文字。
         * @param url 图片完整URL，URL长度不超过1024字节，URL对应的图片base64编码后大小不超过4M，最短边至少15px，最长边最大4096px,支持jpg/png/bmp格式，当image字段存在时url字段失效

         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_language 是否检测语言，默认不检测。当前支持（中文、英语、日语、韩语）
         */
        Json::Value webimage_url(
            std::string const & url,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["url"] = url;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_webimage, null, data, null);

            return result;
        }
        
        /**
         * idcard
         * 用户向服务请求识别身份证，身份证识别包括正面和背面。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * @param id_card_side front：身份证正面；back：身份证背面
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * detect_risk 是否开启身份证风险类型(身份证复印件、临时身份证、身份证翻拍、修改过的身份证)功能，默认不开启，即：false。可选值:true-开启；false-不开启
         */
        Json::Value idcard(
            std::string const & image,
            std::string const & id_card_side,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());
            data["id_card_side"] = id_card_side;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_idcard, null, data, null);

            return result;
        }
        
        /**
         * bankcard
         * 识别银行卡并返回卡号和发卡行。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value bankcard(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_bankcard, null, data, null);

            return result;
        }
        
        /**
         * driving_license
         * 对机动车驾驶证所有关键字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         */
        Json::Value driving_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_driving_license, null, data, null);

            return result;
        }
        
        /**
         * vehicle_license
         * 对机动车行驶证正本所有关键字段进行识别
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * detect_direction 是否检测图像朝向，默认不检测，即：false。朝向是指输入图像是正常方向、逆时针旋转90/180/270度。可选值包括:<br/>- true：检测朝向；<br/>- false：不检测朝向。
         * accuracy normal 使用快速服务，1200ms左右时延；缺省或其它值使用高精度服务，1600ms左右时延
         */
        Json::Value vehicle_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_vehicle_license, null, data, null);

            return result;
        }
        
        /**
         * license_plate
         * 识别机动车车牌，并返回签发地和号牌。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value license_plate(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_license_plate, null, data, null);

            return result;
        }
        
        /**
         * business_license
         * 识别营业执照，并返回关键字段的值，包括单位名称、法人、地址、有效期、证件编号、社会信用代码等。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value business_license(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_business_license, null, data, null);

            return result;
        }
        
        /**
         * receipt
         * 用户向服务请求识别医疗票据、发票、的士票、保险保单等票据类图片中的所有文字，并返回文字在图中的位置信息。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         * recognize_granularity 是否定位单字符位置，big：不定位单字符位置，默认值；small：定位单字符位置
         * probability 是否返回识别结果中每一行的置信度
         * accuracy normal 使用快速服务，1200ms左右时延；缺省或其它值使用高精度服务，1600ms左右时延
         */
        Json::Value receipt(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_receipt, null, data, null);

            return result;
        }
        
        /**
         * table_recognize
         * 自动识别表格线及表格内容，结构化输出表头、表尾及每个单元格的文字内容。表格文字识别接口为异步接口，分为两个API：提交请求接口、获取结果接口。
         * @param image 图像文件二进制内容，可以使用aip::get_file_content函数获取
         * options 可选参数:
         */
        Json::Value table_recognize(
            std::string const & image,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["image"] = base64_encode(image.c_str(), (int) image.size());

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_table_recognize, null, data, null);

            return result;
        }
        
        /**
         * table_result_get
         * 获取表格文字识别结果
         * @param request_id 发送表格文字识别请求时返回的request id
         * options 可选参数:
         * result_type 期望获取结果的类型，取值为“excel”时返回xls文件的地址，取值为“json”时返回json格式的字符串,默认为”excel”
         */
        Json::Value table_result_get(
            std::string const & request_id,
            const std::map<std::string, std::string> & options)
        {
            std::map<std::string, std::string> data;
            
            data["request_id"] = request_id;

            std::copy(options.begin(), options.end(), std::inserter(data, data.end()));

            Json::Value result =
                this->request(_table_result_get, null, data, null);

            return result;
        }
        

    };
}
#endif