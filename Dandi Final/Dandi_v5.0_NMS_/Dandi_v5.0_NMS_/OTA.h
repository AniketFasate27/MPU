#include <WiFi.h>
#include <Update.h>

WiFiClient client2;

// Variables to validate response
long contentLength = 0;
bool isValidContentType = false;

// S3 Bucket Config
String host = "nationalsaltmemorial.in"; // Host => bucket-name.s3.region.amazonaws.com
int ota_port = 80; // Non https. For HTTPS 443. As of today, HTTPS doesn't work.

String bin = "/dandi/dandi_device_3/dandi.bin"; // bin file name with a slash in front.
String ver = "/dandi/dandi_device_3/dandi_test_post_new.php";


// Utility to extract header value from headers
String getHeaderValue(String header, String headerName) {
  return header.substring(strlen(headerName.c_str()));
}

void checkVersion() {
  Serial.println("Connecting to: " + String(host));
  // Connect to S3
  Fn = "FIRMWARE_UPDATE";
  if (client2.connect(host.c_str(), ota_port)) {
    // Connection Succeed.
    Serial.println("Fetching Latest Version: " + String(bin));

    // Get the contents of the bin file
    client2.print(String("GET ") + ver + "?Fn=" + Fn + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client2.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("client2 Timeout !");
        client2.stop();
        return;
      }
    }

    int i = 0;
    while (client2.available()) {
      char c = client2.read();
      Serial.print(c);
      Response[i] = c;
      i = i+1;
    }

    Serial.print("Length of response received is: ");Serial.println(i);
    for(int j = 1; j< i; j++)
    {
      if(Response[j-1] == 'F' && Response[j] == 'W')
      {
         FW_Version_Start = j+3;
      } 
      if(FW_Version_Start > 0)
      {
        if((Response[j] == '*') && (j > FW_Version_Start))
        {
          FW_Version_End = j;  
        } 
      }
    }

    int len = FW_Version_End - FW_Version_Start; 
    for(int k = 0; k < len; k++)
    {
      FW_Version_Num[k] = Response[FW_Version_Start+k]; 
    }

    FW_Version = String(FW_Version_Num);
    Serial.print("The FW Version is: ");Serial.println(FW_Version);
    Latest_FW = FW_Version.toInt();
    Serial.print("The Integer FW Version is: ");Serial.println(Latest_FW);
  
  }
}

// OTA Logic 
void execOTA() {
  Serial.println("Connecting to: " + String(host));
  // Connect to S3
  if (client2.connect(host.c_str(), ota_port)) {
    
    // Fecthing the bin
    Serial.println("Fetching Bin: " + String(bin));

    // Get the contents of the bin file
    client2.print(String("GET ") + bin + " HTTP/1.1\r\n" +
                 "Host: " + host + "\r\n" +
                 "Cache-Control: no-cache\r\n" +
                 "Connection: close\r\n\r\n");

    // Check what is being sent
    //    Serial.print(String("GET ") + bin + " HTTP/1.1\r\n" +
    //                 "Host: " + host + "\r\n" +
    //                 "Cache-Control: no-cache\r\n" +
    //                 "Connection: close\r\n\r\n");

    unsigned long timeout = millis();
    while (client2.available() == 0) {
      if (millis() - timeout > 5000) {
        Serial.println("client2 Timeout !");
        client2.stop();
        return;
      }
    }
    // Once the response is available,
    // check stuff

    /*
       Response Structure
        HTTP/1.1 200 OK
        x-amz-id-2: NVKxnU1aIQMmpGKhSwpCBh8y2JPbak18QLIfE+OiUDOos+7UftZKjtCFqrwsGOZRN5Zee0jpTd0=
        x-amz-request-id: 2D56B47560B764EC
        Date: Wed, 14 Jun 2017 03:33:59 GMT
        Last-Modified: Fri, 02 Jun 2017 14:50:11 GMT
        ETag: "d2afebbaaebc38cd669ce36727152af9"
        Accept-Ranges: bytes
        Content-Type: application/octet-stream
        Content-Length: 357280
        Server: AmazonS3
                                   
        {{BIN FILE CONTENTS}}
    */
    while (client2.available()) {
      // read line till /n
      String line = client2.readStringUntil('\n');
      // remove space, to check if the line is end of headers
      line.trim();

      // if the the line is empty,
      // this is end of headers
      // break the while and feed the
      // remaining `client2` to the
      // Update.writeStream();
      if (!line.length()) {
        //headers ended
        break; // and get the OTA started
      }

      // Check if the HTTP Response is 200
      // else break and Exit Update
      if (line.startsWith("HTTP/1.1")) {
        if (line.indexOf("200") < 0) {
          Serial.println("Got a non 200 status code from server. Exiting OTA Update.");
          break;
        }
      }

      // extract headers here
      // Start with content length
      if (line.startsWith("Content-Length: ")) {
        contentLength = atol((getHeaderValue(line, "Content-Length: ")).c_str());
        Serial.println("Got " + String(contentLength) + " bytes from server");
      }

      // Next, the content type
      if (line.startsWith("Content-Type: ")) {
        String contentType = getHeaderValue(line, "Content-Type: ");
        Serial.println("Got " + contentType + " payload.");
        if (contentType == "application/octet-stream") {
          isValidContentType = true;
        }
      }
    }
  } else {
    // Connect to S3 failed
    // May be try?
    // Probably a choppy network?
    Serial.println("Connection to " + String(host) + " failed. Please check your setup");
    // retry??
    // execOTA();
  }

  // Check what is the contentLength and if content type is `application/octet-stream`
  Serial.println("contentLength : " + String(contentLength) + ", isValidContentType : " + String(isValidContentType));

  // check contentLength and content type
  if (contentLength && isValidContentType) {
    // Check if there is enough to OTA Update
    bool canBegin = Update.begin(contentLength);

    // If yes, begin
    if (canBegin) {
      Serial.println("Begin OTA. This may take 2 - 5 mins to complete. Things might be quite for a while.. Patience!");
      delay(1);
      // No activity would appear on the Serial monitor
      // So be patient. This may take 2 - 5mins to complete
      size_t written = Update.writeStream(client2);
      delay(1);
      if (written == contentLength) {
        Serial.println("Written : " + String(written) + " successfully");
      } else {
        Serial.println("Written only : " + String(written) + "/" + String(contentLength) + ". Retry?" );
        // retry??
        // execOTA();
      }

      if (Update.end()) {
        Serial.println("OTA done!");
        if (Update.isFinished()) {
          Serial.println("Update successfully completed. Rebooting.");
          ESP.restart();
        } else {
          Serial.println("Update not finished? Something went wrong!");
        }
      } else {
        Serial.println("Error Occurred. Error #: " + String(Update.getError()));
      }
    } else {
      // not enough space to begin OTA
      // Understand the partitions and
      // space availability
      Serial.println("Not enough space to begin OTA");
//      client2.flush();
      delay(1000);
      ESP.restart();
    }
  } else {
    Serial.println("There was no content in the response");
//    client2.flush();
    delay(1000);
    ESP.restart();
  }
}
