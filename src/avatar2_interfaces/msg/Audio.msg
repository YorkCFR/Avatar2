# Raw audio input
std_msgs/Header header                # lets know when the data came from
uint32 seq                            # sequence number
string audio                          # wav file as raw bytes
string format                         # WAV_1_44100 or WAV_1_22050 as formats
std_msgs/String text                  # text version of the message
