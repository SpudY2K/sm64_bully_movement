#include <iostream>
#include <fstream>
#include <cmath>

#include "BullyPath.hpp"
#include "Constants.hpp"
#include "SpeedRangeSearch.hpp"
#include "Trig.hpp"
#include "Int128.hpp"

ofstream out_stream;
uint128_t counter = 0;


void build_path(BullyPath &path, int n_frames, int total_frames, float max_offset) {
	float base_dist = path.calculate_current_dist();

	if (n_frames % 2 != 0) {
		float max_speed = max_offset / base_dist;

		for (int i = 0; i < path.speed_ranges.size(); i++) {
			if (path.speed_ranges[i].first > max_speed) {
				break;
			}

			if (path.speed_ranges[i].second != nextafterf(path.speed_ranges[i].first, INFINITY)) {
				//Output solution
				#pragma omp critical 
				{
					out_stream << n_frames << "," << path.start_yaw << "," << fixed << path.start_position[0] << "," 
            << fixed << path.start_position[1] << "," << fixed << path.start_position[2] << "," 
            << fixed << path.speed_ranges[i].first << "," << fixed << path.speed_ranges[i].second << "," 
            << fixed << (base_dist*path.speed_ranges[i].first) << "," 
            << fixed << (base_dist*path.speed_ranges[i].second) << "\n";
          counter++;
				}
			}
		}
	}

	if (n_frames > total_frames || path.speed_ranges.empty() || (base_dist - (total_frames - n_frames)*fabs(path.current_direction))*path.speed_ranges[0].first > max_offset) {
		return;
	}
	else {
		vector<float> next_position = path.path_positions.back();
		next_position[next_position.size() - 1] += path.current_direction;

		int next_pos_idx = -1;

		for (int i = 0; i < path.checked_positions.size(); i++) {
			if (next_position == path.checked_positions[i]) {
				next_pos_idx = i;
				break;
			}
		}

		bool stop_falling = false;

		BullyPath new_path;

		if (next_pos_idx == -1 || path.checked_states[next_pos_idx] == STATE_CLEAR || path.checked_states[next_pos_idx] == STATE_START) {
			new_path = path;

			if (next_pos_idx == -1) {
				new_path.checked_positions.push_back(next_position);
				new_path.checked_states.push_back(STATE_CLEAR);
				new_path.update_speed_ranges(next_position, STATE_CLEAR, -1);
			}
			else if (path.checked_states[next_pos_idx] == STATE_START) {
				new_path.checked_states[next_pos_idx] = STATE_CLEAR;
			}


			if (new_path.falling) {
				new_path.falling_frames++;
			}

			if (new_path.falling && new_path.start_position[1] - bully_gravity * (float)(new_path.falling_frames + 1)*(float)(new_path.falling_frames + 2) / 2.0f < lava_y + 5) {
				new_path.falling = false;
				stop_falling = true;
			}
			else {
				if (!new_path.speed_ranges.empty()) {
					new_path.path_positions.push_back(next_position);

					build_path(new_path, n_frames + 1, total_frames, max_offset);
				}
			}
		}

		if (next_pos_idx == -1 || path.checked_states[next_pos_idx] == STATE_OOB) {
			new_path = path;

			if (next_pos_idx == -1) {
				new_path.checked_positions.push_back(next_position);
				new_path.checked_states.push_back(STATE_OOB);
				new_path.update_speed_ranges(next_position, STATE_OOB, -1);
			}

			if (!new_path.speed_ranges.empty()) {
				if (path.current_yaw % 16 == 0) {
					for (int i = 0; i < new_path.path_positions.size(); i++) {
						new_path.path_positions[i].push_back({ 0.0 });
					}

					for (int i = 0; i < new_path.checked_positions.size(); i++) {
						new_path.checked_positions[i].push_back({ 0.0 });
					}

					vector<float> true_next_position = path.path_positions.back();

					new_path.current_direction = 1.0;
					new_path.current_yaw += 32767;

					true_next_position.push_back(new_path.current_direction);
					new_path.path_positions.push_back(true_next_position);

					int next_hau = (uint16_t)new_path.current_yaw >> 4;
					new_path.vector_haus.push_back(next_hau);

					new_path.vector_lengths.push_back(new_path.current_length);

					build_path(new_path, n_frames + 1, total_frames, max_offset);
				}
				else {
					vector<float> true_next_position = path.path_positions.back();

					new_path.current_direction = -new_path.current_direction;
					new_path.current_yaw += 32767;

					true_next_position[next_position.size() - 1] += new_path.current_direction;
					new_path.path_positions.push_back(true_next_position);

					build_path(new_path, n_frames + 1, total_frames, max_offset);
				}
			}
		}

		if (next_pos_idx == -1 || (path.checked_states[next_pos_idx] & 0x111) == STATE_STEEP_FLOOR) {
			int start; int end;

			if (next_pos_idx == -1) {
				start = 0;
				end = steep_floors.size();
			}
			else {
				start = path.checked_states[next_pos_idx] >> 3;
				end = start + 1;
			}

			for (int h = start; h < end; h++) {
				new_path = path;

				if (next_pos_idx == -1) {
					new_path.checked_positions.push_back(next_position);
					new_path.checked_states.push_back(STATE_STEEP_FLOOR | (h << 3));
					new_path.update_speed_ranges(next_position, STATE_STEEP_FLOOR, h);
				}

				if (!new_path.speed_ranges.empty()) {
					for (int i = 0; i < new_path.path_positions.size(); i++) {
						new_path.path_positions[i].push_back({ 0.0 });
					}

					for (int i = 0; i < new_path.checked_positions.size(); i++) {
						new_path.checked_positions[i].push_back({ 0.0 });
					}

					new_path.current_direction = 1.0;

					vector<float> true_next_position = path.path_positions.back();
					true_next_position.push_back(new_path.current_direction);
					new_path.path_positions.push_back(true_next_position);

					new_path.current_yaw = (new_path.current_yaw >> 4) << 4;
					new_path.current_yaw = steep_floors[h].first - new_path.current_yaw + 32768;
					int next_hau = (uint16_t)new_path.current_yaw >> 4;
					new_path.current_yaw = atan2s(gCosineTable[next_hau], gSineTable[next_hau]);
					next_hau = (uint16_t)new_path.current_yaw >> 4;
					new_path.vector_haus.push_back(next_hau);

					new_path.vector_lengths.push_back(new_path.current_length);

					build_path(new_path, n_frames + 1, total_frames, max_offset);
				}
			}
		}

		if (next_pos_idx == -1 || (path.checked_states[next_pos_idx] & 0x111) == STATE_WALL) {
			int start; int end;

			if (next_pos_idx == -1) {
				start = 0;
				end = walls.size();
			}
			else {
				start = path.checked_states[next_pos_idx] >> 3;
				end = start + 1;
			}

			for (int h = start; h < end; h++) {
				new_path = path;

				if (next_pos_idx == -1) {
					new_path.checked_positions.push_back(next_position);
					new_path.checked_states.push_back(STATE_WALL | (h << 3));
					new_path.update_speed_ranges(next_position, STATE_WALL, h);
				}


				if (new_path.falling) {
					new_path.falling_frames++;
				}

				if (new_path.falling && new_path.start_position[1] - bully_gravity * (float)(new_path.falling_frames + 1)*(float)(new_path.falling_frames + 2) / 2.0f < lava_y + 5) {
					new_path.falling = false;
					stop_falling = true;
				}
				else {
					if (!new_path.speed_ranges.empty()) {
						for (int i = 0; i < new_path.path_positions.size(); i++) {
							new_path.path_positions[i].push_back({ 0.0 });
						}

						for (int i = 0; i < new_path.checked_positions.size(); i++) {
							new_path.checked_positions[i].push_back({ 0.0 });
						}

						new_path.current_direction = 1.0;

						vector<float> true_next_position = next_position;
						true_next_position.push_back(new_path.current_direction);
						new_path.path_positions.push_back(true_next_position);

						new_path.current_yaw = (new_path.current_yaw >> 4) << 4;
						new_path.current_yaw = walls[h].first - new_path.current_yaw + 32768;
						int next_hau = (uint16_t)new_path.current_yaw >> 4;
						new_path.current_yaw = atan2s(gCosineTable[next_hau], gSineTable[next_hau]);
						next_hau = (uint16_t)new_path.current_yaw >> 4;
						new_path.vector_haus.push_back(next_hau);

						new_path.vector_lengths.push_back(new_path.current_length);

						build_path(new_path, n_frames + 1, total_frames, max_offset);
					}
				}
			}
		}

		if (next_pos_idx == -1 || path.checked_states[next_pos_idx] == STATE_FRICTION_FLOOR || path.checked_states[next_pos_idx] == STATE_START || stop_falling) {
			new_path = path;

			if (!stop_falling) {
				if (next_pos_idx == -1) {
					new_path.falling_frames++;
					new_path.checked_positions.push_back(next_position);
					new_path.checked_states.push_back(STATE_FRICTION_FLOOR);
					new_path.update_speed_ranges(next_position, STATE_FRICTION_FLOOR, -1);
				}
				else if (path.checked_states[next_pos_idx] == STATE_START) {
					new_path.checked_states[next_pos_idx] = STATE_FRICTION_FLOOR;
				}
			}

			if (!new_path.speed_ranges.empty()) {
				new_path.falling = false; //Change to allow falling after friction?

				new_path.current_length = new_path.current_length*bully_friction;

				int current_hau = (uint16_t)new_path.current_yaw >> 4;
				new_path.current_yaw = atan2s(gCosineTable[current_hau], gSineTable[current_hau]);
				int next_hau = (uint16_t)new_path.current_yaw >> 4;

				for (int i = 0; i < new_path.path_positions.size(); i++) {
					new_path.path_positions[i].push_back({ 0.0 });
				}

				for (int i = 0; i < new_path.checked_positions.size(); i++) {
					new_path.checked_positions[i].push_back({ 0.0 });
				}

				new_path.current_direction = 1.0;

				vector<float> true_next_position = path.path_positions.back();
				true_next_position.push_back(new_path.current_direction);
				new_path.path_positions.push_back(true_next_position);
				new_path.vector_haus.push_back(next_hau);

				new_path.vector_lengths.push_back(new_path.current_length);

				build_path(new_path, n_frames + 1, total_frames, max_offset);
			}
		}
	}
}

void find_paths(vector<float> &start_position, float min_speed, float max_speed, int total_frames, float max_offset) {
	/*
	for (int i = 0; i < 65536; i++) {
		BullyPath current_path(start_position, i, min_speed, max_speed);
		build_path(current_path, total_frames);
	}
	*/
	
	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 1024; i++) {
		cout << gArctanTable[i] << "\n";
		BullyPath current_path(start_position, gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 1024; i > 0; i--) {
		cout << (0x4000 - gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0x4000 - gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 1024; i++) {
		cout << (0x4000 + gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0x4000 + gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 1024; i > 0; i--) {
		cout << (0x8000 - gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0x8000 - gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 1024; i++) {
		cout << (0x8000 + gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0x8000 + gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 1024; i > 0; i--) {
		cout << (0xC000 - gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0xC000 - gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 0; i < 1024; i++) {
		cout << (0xC000 + gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, 0xC000 + gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}

	#pragma omp parallel for schedule(dynamic, 1)
	for (int i = 1024; i > 0; i--) {
		cout << (-gArctanTable[i]) << "\n";
		BullyPath current_path(start_position, -gArctanTable[i], min_speed, max_speed);
		build_path(current_path, 0, total_frames, max_offset);
	}
}

int main()
{
	int total_frames = 26;
	float min_speed = 1000000.0f;
	float max_speed = 1000000000.0f;
	float max_offset = 1000.0f;

	vector<float> start_position = { -2236.0f, -2950.0f, -566.0f };

	out_stream.open("BullyPositions.txt");

	find_paths(start_position, min_speed, max_speed, total_frames, max_offset);
  
  // RAII will autoclose out_stream, no need to worry about it
	// out_stream.close();
  cout << "Found " << to_string(counter) << " solutions" << endl;
}
