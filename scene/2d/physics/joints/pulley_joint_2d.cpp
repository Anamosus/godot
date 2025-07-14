/**************************************************************************/
/*  pulley_joint_2d.cpp                                            */
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

#include "pulley_joint_2d.h"
#include "scene/2d/node_2d.h"
#include "scene/2d/physics/physics_body_2d.h"

void PulleyJoint2D::_notification(int p_what) {
	switch (p_what) {

		case NOTIFICATION_READY: 
			set_process(true);
			break;

		case NOTIFICATION_PROCESS: {
			Node *node_o = get_node_or_null(other_anchor);
			Node2D *anchor_o = Object::cast_to<Node2D>(node_o);
			if (anchor_o) {				
				Transform2D temp_transform = get_global_transform().affine_inverse() * anchor_o->get_global_transform();
				if (temp_transform != other_transform) {
					other_transform = temp_transform;
					queue_redraw();
				}
			}

			if (!node_o) {
				warning = RTR("Other anchor must be assinged");				
			} else if (!anchor_o) {
				warning = RTR("Other anchor must be a Node2D");				
			} else {
				warning = String();
			}	
			break;
		}			

		case NOTIFICATION_DRAW: {
			if (!is_inside_tree()) {
				break;
			}

			if (!Engine::get_singleton()->is_editor_hint() && !get_tree()->is_debugging_collisions_hint()) {
				break;
			}

			draw_line(Point2(-10, 0), Point2(+10, 0), Color(0.7, 0.6, 0.0, 0.5), 3);
			draw_line(Point2(-10, length_a), Point2(+10, length_a), Color(0.7, 0.6, 0.0, 0.5), 3);
			draw_line(Point2(0, 0), Point2(0, length_a), Color(0.7, 0.6, 0.0, 0.5), 3);

			if (get_node_or_null(other_anchor)) {
				draw_set_transform_matrix(other_transform);
				draw_line(Point2(-10, 0), Point2(+10, 0), Color(0.7, 0.6, 0.0, 0.5), 3);
				draw_line(Point2(-10, length_b), Point2(+10, length_b), Color(0.7, 0.6, 0.0, 0.5), 3);
				draw_line(Point2(0, 0), Point2(0, length_b), Color(0.7, 0.6, 0.0, 0.5), 3);
			}
			break;
		}		
	}
}

void PulleyJoint2D::_configure_joint(RID p_joint, PhysicsBody2D *body_a, PhysicsBody2D *body_b) {
	Node *node_o = get_node_or_null(other_anchor);
	Node2D *anchor_o = Object::cast_to<Node2D>(node_o);

	if (!anchor_o) {
		return;
	}

	Transform2D gtA = get_global_transform();
	Vector2 anchor_A = gtA.get_origin();
	Vector2 ground_anchor_A = gtA.xform(Vector2(0, length_a));

	Transform2D gtB = anchor_o->get_global_transform();
	Vector2 anchor_B = gtB.get_origin();
	Vector2 connected_anchor_B = gtB.xform(Vector2(0, length_b));

	PhysicsServer2D::get_singleton()->joint_make_pulley(p_joint, anchor_A, anchor_B, ground_anchor_A, connected_anchor_B, body_a->get_rid(), body_b->get_rid());
	if (max_length) {
		PhysicsServer2D::get_singleton()->pulley_joint_set_param(p_joint, PhysicsServer2D::PULLEY_MAX_LENGTH, max_length);
	}
	PhysicsServer2D::get_singleton()->pulley_joint_set_param(p_joint, PhysicsServer2D::PULLEY_MIN_LENGTH, min_length);

	PhysicsServer2D::get_singleton()->pulley_joint_set_param(p_joint, PhysicsServer2D::PULLEY_STIFFNESS, stiffness);
	PhysicsServer2D::get_singleton()->pulley_joint_set_param(p_joint, PhysicsServer2D::PULLEY_DAMPING, damping);
}

PackedStringArray PulleyJoint2D::get_configuration_warnings() const {
	PackedStringArray warnings = Joint2D::get_configuration_warnings();

	if (!warning.is_empty()) {
		warnings.push_back(warning);
	}
	return warnings;
}

void PulleyJoint2D::set_length_a(real_t p_length) {
	length_a = p_length;
	queue_redraw();
}

real_t PulleyJoint2D::get_length_a() const {
	return length_a;
}


void PulleyJoint2D::set_length_b(real_t p_length) {
	length_b = p_length;
	queue_redraw();
}

real_t PulleyJoint2D::get_length_b() const {
	return length_b;
}


void PulleyJoint2D::set_min_length(real_t p_length) {
	min_length = p_length;
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->pulley_joint_set_param(get_rid(), PhysicsServer2D::PULLEY_MIN_LENGTH, p_length);
	}
}

real_t PulleyJoint2D::get_min_length() const {
	return min_length;
}

void PulleyJoint2D::set_max_length(real_t p_length) {
	max_length = p_length;	
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->pulley_joint_set_param(get_rid(), PhysicsServer2D::PULLEY_MAX_LENGTH, p_length);
	}
}

real_t PulleyJoint2D::get_max_length() const {
	if (is_configured() && !Engine::get_singleton()->is_editor_hint()) {
		return PhysicsServer2D::get_singleton()->pulley_joint_get_param(get_rid(), PhysicsServer2D::PULLEY_MAX_LENGTH);
	}
	return max_length;
}

void PulleyJoint2D::set_stiffness(real_t p_stiffness) {
	stiffness = p_stiffness;
	if (is_configured()) {
		PhysicsServer2D::get_singleton()->pulley_joint_set_param(get_rid(), PhysicsServer2D::PULLEY_STIFFNESS, p_stiffness);
	}
}

real_t PulleyJoint2D::get_stiffness() const {
	return stiffness;
}

void PulleyJoint2D::set_damping(real_t p_damping) {
	damping = p_damping;
	if (is_configured()) {	
		PhysicsServer2D::get_singleton()->pulley_joint_set_param(get_rid(), PhysicsServer2D::PULLEY_DAMPING, p_damping);
	}
}

real_t PulleyJoint2D::get_damping() const {
	return damping;
}

void PulleyJoint2D::set_other_anchor(const NodePath &p_other_anchor) {
	if (other_anchor == p_other_anchor) {
		return;
	}
	other_anchor = p_other_anchor;
}

NodePath PulleyJoint2D::get_other_anchor() const {
	return other_anchor;
}

bool PulleyJoint2D::is_fixed_distance() const {
	return fixed_length;
}

void PulleyJoint2D::set_fixed_distance(bool p_fixed_distance) {
	fixed_length = p_fixed_distance;
}

void PulleyJoint2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("set_length_a", "length_a"), &PulleyJoint2D::set_length_a);
	ClassDB::bind_method(D_METHOD("get_length_a"), &PulleyJoint2D::get_length_a);
	ClassDB::bind_method(D_METHOD("set_length_b", "length_b"), &PulleyJoint2D::set_length_b);
	ClassDB::bind_method(D_METHOD("get_length_b"), &PulleyJoint2D::get_length_b);
	ClassDB::bind_method(D_METHOD("set_max_length", "max_length"), &PulleyJoint2D::set_max_length);
	ClassDB::bind_method(D_METHOD("get_max_length"), &PulleyJoint2D::get_max_length);
	ClassDB::bind_method(D_METHOD("set_min_length", "min_length"), &PulleyJoint2D::set_min_length);
	ClassDB::bind_method(D_METHOD("get_min_length"), &PulleyJoint2D::get_min_length);
	ClassDB::bind_method(D_METHOD("set_stiffness", "stiffness"), &PulleyJoint2D::set_stiffness);
	ClassDB::bind_method(D_METHOD("get_stiffness"), &PulleyJoint2D::get_stiffness);
	ClassDB::bind_method(D_METHOD("set_damping", "damping"), &PulleyJoint2D::set_damping);
	ClassDB::bind_method(D_METHOD("get_damping"), &PulleyJoint2D::get_damping);
	ClassDB::bind_method(D_METHOD("set_other_anchor", "node"), &PulleyJoint2D::set_other_anchor);
	ClassDB::bind_method(D_METHOD("get_other_anchor"), &PulleyJoint2D::get_other_anchor);

	ClassDB::bind_method(D_METHOD("set_fixed_distance", "fixed_distance"), &PulleyJoint2D::set_fixed_distance);
	ClassDB::bind_method(D_METHOD("is_fixed_distance"), &PulleyJoint2D::is_fixed_distance);

	ADD_PROPERTY(PropertyInfo(Variant::NODE_PATH, "other_anchor", PROPERTY_HINT_NODE_PATH_VALID_TYPES, "Node2D"), "set_other_anchor", "get_other_anchor");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length_a", PROPERTY_HINT_RANGE, "1,65535,1,exp,suffix:px"), "set_length_a", "get_length_a");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "length_b", PROPERTY_HINT_RANGE, "1,65535,1,exp,suffix:px"), "set_length_b", "get_length_b");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "min_length", PROPERTY_HINT_RANGE, "0,65535,1,exp,suffix:px"), "set_min_length", "get_min_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "max_length", PROPERTY_HINT_RANGE, "0,65535,1,exp,suffix:px"), "set_max_length", "get_max_length");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "stiffness", PROPERTY_HINT_RANGE, "0.1,64,0.1,exp"), "set_stiffness", "get_stiffness");
	ADD_PROPERTY(PropertyInfo(Variant::FLOAT, "damping", PROPERTY_HINT_RANGE, "0.01,16,0.01,exp"), "set_damping", "get_damping");

	ADD_PROPERTY(PropertyInfo(Variant::BOOL, "fixed_distance"), "set_fixed_distance", "is_fixed_distance");
}

PulleyJoint2D::PulleyJoint2D() {
}
