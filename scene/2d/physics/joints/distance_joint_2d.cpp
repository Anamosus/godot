/**************************************************************************/
/*  distance_joint_2d.cpp                                            */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#include "distance_joint_2d.h"

#include "scene/2d/physics/physics_body_2d.h"

void DistanceJoint2D::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_DRAW: {
			if (!is_inside_tree()) {
				break;
			}

			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			draw_line(Point2(-10, 0), Point2(+10, 0), Color(0.7, 0.6, 0.0, 0.5), 3);
			draw_line(Point2(-10, length), Point2(+10, length), Color(0.7, 0.6, 0.0, 0.5), 3);
			draw_line(Point2(0, 0), Point2(0, length), Color(0.7, 0.6, 0.0, 0.5), 3);
		} break;
	}
}

void DistanceJoint2D::_configure_joint(RID p_joint, PhysicsBody2D *body_a, PhysicsBody2D *body_b) {
	Transform2D gt = get_global_transform();
	Vector2 anchor_A = gt.get_origin();
	Vector2 anchor_B = gt.xform(Vector2(0, length));

	PhysicsServer2D::get_singleton()->joint_make_distance(p_joint, anchor_A, anchor_B, body_a->get_rid(), body_b->get_rid());
	PhysicsServer2D::get_singleton()->distance_joint_set_param(p_joint, PhysicsServer2D::DISTANCE_MIN_LENGTH, min_length);
	if (max_length) {
		PhysicsServer2D::get_singleton()->distance_joint_set_param(p_joint, PhysicsServer2D::DISTANCE_MAX_LENGTH, max_length);
	}

	PhysicsServer2D::get_singleton()->distance_joint_set_param(p_joint, PhysicsServer2D::DISTANCE_SPRING_FREQUENCY, stiffness);
	PhysicsServer2D::get_singleton()->distance_joint_set_param(p_joint, PhysicsServer2D::DISTANCE_SPRING_DAMPING, damping);
	PhysicsServer2D::get_singleton()->distance_joint_set_flag(p_joint, PhysicsServer2D::DISTANCE_FIXED_LENGTH, fixed_length);
}

void DistanceJoint2D::set_length(real_t p_length) {
	length = p_length;
	queue_redraw();
}

real_t DistanceJoint2D::get_length() const {
	return length;
}

void DistanceJoint2D::set_min_length(real_t p_length) {	
	min_length = p_length;

	if (is_configured()) {
		PhysicsServer2D::get_singleton()->distance_joint_set_param(get_rid(), PhysicsServer2D::DISTANCE_MIN_LENGTH, min_length);
	}
}

real_t DistanceJoint2D::get_min_length() const {
	return min_length;
}

void DistanceJoint2D::set_max_length(real_t p_length) {
	max_length = p_length;

	if (is_configured()) {
		PhysicsServer2D::get_singleton()->distance_joint_set_param(get_rid(), PhysicsServer2D::DISTANCE_MAX_LENGTH, max_length);
	}
}

real_t DistanceJoint2D::get_max_length() const {
	return max_length;
}

void DistanceJoint2D::set_stiffness(real_t p_stiffness) {
	stiffness = p_stiffness;
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->distance_joint_set_param(get_rid(), PhysicsServer2D::DISTANCE_SPRING_FREQUENCY, p_stiffness);
	}
}

real_t DistanceJoint2D::get_stiffness() const {
	return stiffness;
}

void DistanceJoint2D::set_damping(real_t p_damping) {
	damping = p_damping;
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->distance_joint_set_param(get_rid(), PhysicsServer2D::DISTANCE_SPRING_DAMPING, p_damping);
	}
}

bool DistanceJoint2D::is_fixed_distance() const {
	return fixed_length;
}

void DistanceJoint2D::set_fixed_distance(bool p_fixed_distance) {
	fixed_length = p_fixed_distance;
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->distance_joint_set_flag(get_rid(), PhysicsServer2D::DISTANCE_FIXED_LENGTH, p_fixed_distance);
	}
}


real_t DistanceJoint2D::get_damping() const {
	return damping;
}


void DistanceJoint2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_length", "length"), &DistanceJoint2D::set_length);
	ClassDB::bind_method(D_METHOD("get_length"), &DistanceJoint2D::get_length);

	ClassDB::bind_method(D_METHOD("set_min_length", "min_length"), &DistanceJoint2D::set_min_length);
	ClassDB::bind_method(D_METHOD("get_min_length"), &DistanceJoint2D::get_min_length);

	ClassDB::bind_method(D_METHOD("set_max_length", "max_length"), &DistanceJoint2D::set_max_length);
	ClassDB::bind_method(D_METHOD("get_max_length"), &DistanceJoint2D::get_max_length);

	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &DistanceJoint2D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &DistanceJoint2D::get_stiffness);

	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &DistanceJoint2D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &DistanceJoint2D::get_damping);	

	ClassDB::bind_method(D_METHOD("set_fixed_distance", "fixed_distance"), &DistanceJoint2D::set_fixed_distance);
	ClassDB::bind_method(D_METHOD("is_fixed_distance"), &DistanceJoint2D::is_fixed_distance);



	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length", PROPERTY_HINT_RANGE, "1,65535,1,exp,suffix:px"), "set_length", "get_length");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_length", PROPERTY_HINT_RANGE, "0,65535,1,exp,suffix:px"), "set_min_length", "get_min_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_length", PROPERTY_HINT_RANGE, "0,65535,1,exp,suffix:px"), "set_max_length", "get_max_length");

	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_stiffness", PROPERTY_HINT_RANGE, "0.0,16,.01,exp"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "spring_damping", PROPERTY_HINT_RANGE, "0.01,2,.01,exp"), "set_damping", "get_damping");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fixed_distance"), "set_fixed_distance", "is_fixed_distance");
}

DistanceJoint2D::DistanceJoint2D() {
}
