#include <HTTPClient.h>

#ifndef WEBCLIENT_H
#define WEBCLIENT_H

class WebClient {
    public:
        void handleData(float temp, float ph, float ec);
    private:
        String address = "http://10.42.0.1";
};


#endif // WEBCLIENT_H