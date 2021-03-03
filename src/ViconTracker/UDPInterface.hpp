#ifndef VICONTRACKER_UDP_HPP
#define VICONTRACKER_UDP_HPP

#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace ViconTracker {

enum ItemID {
	Object = 0,
	};

struct __attribute__ ((packed)) TrackerObject_raw {
	char ItemName[24];
	double TransX;
	double TransY;
	double TransZ;
	double RotX;
	double RotY;
	double RotZ;
	};

struct __attribute__((packed)) ItemHeader_raw {
	uint8_t ItemID;
	uint16_t ItemDataSize;
	};

struct __attribute__((packed)) Item_raw {
	ItemHeader_raw Header;
	uint8_t Data[];
	};

struct __attribute__((packed)) UDPStreamPacket_raw {
	uint32_t FrameNumber;
	uint8_t ItemsInBlock;
	Item_raw Items[];
	};

class UDPStreamPacket {
	public:
		UDPStreamPacket(unsigned char const* const buffer, int const buffer_size) {
			auto packetObj = reinterpret_cast<UDPStreamPacket_raw const*>(buffer);
			uint8_t numItems = packetObj->ItemsInBlock;
			size_t itemOffset = 0;
			for( int i = 0; i < numItems; i++ ) {
				auto itemObj = reinterpret_cast<Item_raw const*>(
					reinterpret_cast<uint8_t const*>(packetObj->Items) + itemOffset // reinterpret_cast here to allow byte-value pointer arithmetic
					);
				if( itemObj->Header.ItemID == ItemID::Object ) {
					auto trackerObj = reinterpret_cast<TrackerObject_raw const*>(itemObj->Data);
					objects.push_back(trackerObj);
					}
				// If unknown item type, just move to next item
				itemOffset += sizeof(ItemHeader_raw) + itemObj->Header.ItemDataSize;
				}
			}
		
		std::vector<TrackerObject_raw const*> objects;
	};

};

#endif//VICONTRACKER_UDP_HPP
