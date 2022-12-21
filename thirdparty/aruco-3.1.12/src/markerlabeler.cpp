/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.
Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:
   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.
   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.
THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/
#include "markerlabeler.h"
#include "dictionary_based.h"
 namespace aruco
{
    cv::Ptr<MarkerLabeler> MarkerLabeler::create(Dictionary::DICT_TYPES dict_type,
                                                 float error_correction_rate)
    {
        Dictionary dict = Dictionary::loadPredefined(dict_type);
        DictionaryBased* db = new DictionaryBased();
        db->setParams(dict, error_correction_rate);
        return db;
    }
    cv::Ptr<MarkerLabeler> MarkerLabeler::create(std::string detector, std::string params)
    {
        auto _stof=[](std::string str){
            float f;sscanf(str.c_str(),"%f",&f);return f;
        };
        (void)params;
        Dictionary dict = Dictionary::load(detector);
        // try with one from file
        DictionaryBased* db = new DictionaryBased();
        db->setParams(dict, _stof(params));
        return db;
    }
 }
