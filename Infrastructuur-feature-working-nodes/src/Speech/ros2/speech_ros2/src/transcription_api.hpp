#ifndef transcribe_api
#define transcribe_api

#include <curl/curl.h>
#include <string>
#include <chrono>

#include "json.hpp"

using namespace std::chrono;

/**
 * @brief allows us to recieve the response of CURL into a string
 */
size_t writeCallback(char* contents, size_t size, size_t nmemb, void* userp) {
    ((std::string*)userp)->append((char*)contents, size * nmemb);
    return size * nmemb;
}

/**
 * @brief class for sending audio to MS STT and recieving transcription
 */
class transcriptionAPI {
private:
    time_point<steady_clock> issueTime; /**< timestamp the token got issued */
    int expiryTime = 60000000;		/**< how long the token is valid in microseconds */
    std::string token;			/**< the token itself */
public:
    /**
     * @brief constructor for transcription API
     * @details initializes the class by getting a token
     */
    transcriptionAPI() {
        getToken();
    }

    /**
     * @brief issues a new token from MS API
     * @details gets a new token and sets the timestamp to the current time
     */
    void getToken() {
        std::string _token;
        struct curl_slist* chunk = NULL;
        issueTime = steady_clock::now();

        chunk = curl_slist_append(chunk, "Content-type: application/x-www-form-urlencoded");
        chunk = curl_slist_append(chunk, "Content-Length: 0");
        chunk = curl_slist_append(chunk, "Ocp-Apim-Subscription-Key: 162470352a474535bf355e00b6e41870");

        CURL* curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_URL, "https://westeurope.api.cognitive.microsoft.com/sts/v1.0/issueToken");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &_token);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, "");

        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_slist_free_all(chunk);

        token = _token;
    }

    /**
     * @brief checks if the token has expired and issues a new one if so
     */
    void checkToken() {
        time_point<steady_clock> end = steady_clock::now();
        long long deltaTime = duration_cast<microseconds>(end - issueTime).count();

        if (deltaTime > expiryTime) {
            getToken();
        }
    }

    /**
     * @brief transcribes the audio to text
     * @param buffer, buffer containing playable audio file
     * @param size, size of the input buffer
     * @details sends the buffer to the MS STT and waits for the transcription
     * @return returns a string with json containing the transcription
     */
    std::string transcribeAudio(char* buffer, int size) {
        struct curl_slist* chunk = NULL;
        std::string transcription = "", auth;
        checkToken();

        auth = "Authorization: Bearer " + token;
        chunk = curl_slist_append(chunk, auth.c_str());
        chunk = curl_slist_append(chunk, "Content-type: audio/wav; codec=audio/pcm; samplerate=16000");

        CURL* curl = curl_easy_init();
        curl_easy_setopt(curl, CURLOPT_URL, "https://westeurope.stt.speech.microsoft.com/speech/recognition/conversation/cognitiveservices/v1?language=nl-NL");
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, writeCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &transcription);
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, chunk);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, size);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, buffer);

        curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        curl_slist_free_all(chunk);

	nlohmann::json obj = nlohmann::json::parse(transcription);
	if(obj["RecognitionStatus"].get<std::string>() == "Success") {
	    return obj["DisplayText"].get<std::string>();
	}
	return "Transcription failed";
    }
};

#endif // transcribe_api
