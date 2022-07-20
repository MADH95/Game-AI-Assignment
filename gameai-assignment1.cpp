#include "vec.hpp"
#include "draw-triangle-pro.hpp"
#include "raylib-cpp.hpp"
#include <vector>
#include <cmath>  // std::atan, std::fmod, std::abs
#include <random> // std::random_device, std::mt19937
#include <algorithm> // std::clamp
#include <variant> // std::variant, std::visit

using Vector = ai::Vector3;  // use x and z in the 2D case

constexpr int WINDOW_WIDTH = 1024, WINDOW_HEIGHT = 768;

std::random_device rd;
std::mt19937 seed( rd() );

// These two values each represent an acceleration.
// i.e. they effect changes in velocity (linear and angular).
class SteeringOutput
{
public:
	Vector linear_;
	float angular_;

	SteeringOutput &operator+=( const SteeringOutput &rhs )
	{
		linear_ += rhs.linear_;
		angular_ += rhs.angular_;
		return *this;
	}
	friend SteeringOutput operator*( const float lhs, const SteeringOutput &y )
	{
		return { lhs * y.linear_, lhs * y.angular_ };
	}
};

class Kinematic
{
public:
	Vector position_;
	float orientation_;
	Vector velocity_;
	float rotation_;

	// integration of the linear and angular accelerations
	void update( const SteeringOutput &steering,
				 const float maxSpeed,
				 float drag,
				 const float time ) // delta time
	{
		//Newton-Euler 1 simplification:
		position_ += velocity_ * time;
		orientation_ += rotation_ * time;
		orientation_ = std::fmod( orientation_, 2 * PI ); // (-2pi,2pi) - not crucial?

		velocity_ += steering.linear_ * time;
		rotation_ += steering.angular_ * time;

		post_process( drag, maxSpeed, time );
	}

	void post_process( const float drag, const float maxSpeed, const float time )
	{
		velocity_ *= ( 1 - drag * time );
		rotation_ *= ( 1 - drag * time );

		if ( velocity_.length() > maxSpeed )
		{
			velocity_.normalise();
			velocity_ *= maxSpeed;
		}

		//Screen Wrapping
		if ( position_.x > WINDOW_HEIGHT )
			position_.x -= WINDOW_HEIGHT;
		else if ( position_.x < 0 )
			position_.x += WINDOW_HEIGHT;

		if ( position_.z > WINDOW_WIDTH )
			position_.z -= WINDOW_WIDTH;
		else if ( position_.z < 0 )
			position_.z += WINDOW_WIDTH;
	}
};

class Ship
{
public:
	Ship( const float z, const float x, const float ori, const raylib::Color col )
		: k_{ {x,0,z},ori,{0,0,0},0 }, col_{ col } { }

	Kinematic k_;
	raylib::Color col_;

	void draw( int screenwidth, int screenheight )
	{
		const float w = 10, hlen = 15, o = 2.0f; // ship width and length
		const ai::Vector2 l{ -hlen, -w }, r{ -hlen, w }, nose{ hlen, 0 };
		ai::Vector2 pos{ k_.position_.z, k_.position_.x };
		float ori = -k_.orientation_ * RAD2DEG; // negate: anticlockwise rot

		ai::DrawTrianglePro( pos, l, r, nose, ori, col_ );
	}
};

// Dynamic Seek (page 96)
class Seek
{
public:
	Kinematic &character_;
	Kinematic &target_;

	float maxAcceleration_;

	/* // A constructor isn't needed, but adding it will also not hurt
	Seek(Kinematic &c, Kinematic &t, float maxAcceleration)
	  : character_{c}, target_{t}, maxAcceleration{maxAcceleration_}
	{
	}*/

	SteeringOutput getSteering() const
	{
		SteeringOutput result;

		result.linear_ = target_.position_ - character_.position_;

		result.linear_.normalise();
		result.linear_ *= maxAcceleration_;

		result.angular_ = 0;
		return result;
	}
};

class Wander
{
public:
	Kinematic &character_;
	float maxSpeed_;
	float maxRotation_;

	SteeringOutput getSteering() const
	{
		SteeringOutput result;

		result.linear_ = maxSpeed_ * ai::asVector( character_.orientation_ );

		result.angular_ = randomBinomial() * maxRotation_;

		return result;
	}

	float randomBinomial() const
	{
		std::uniform_real_distribution<float> dis( 0.0f, 1.0f );
		return dis( seed ) - dis( seed );
	}
};

class Align
{
public:
	Kinematic &character_;
	Kinematic &target_;

	float maxAngularAccel_;
	float maxRotation_;

	float targetRadius_;
	float slowRadius_;

	float timeToTarget_ = 0.1f;

	SteeringOutput getSteering() const
	{
		SteeringOutput result{};


		//This should be in post_process, but that breaks wander.
		if ( character_.velocity_.length() > 0 )
			character_.orientation_ = std::atan2( -character_.velocity_.x, character_.velocity_.z );

		float rotation = target_.orientation_ - character_.orientation_;

		rotation = std::abs( rotation ) > PI ? rotation - ( 2 * PI ) : rotation;
		float rotationSize = std::abs( rotation );

		if ( rotationSize < targetRadius_ )
			return result;

		float targetRotation;
		if ( rotationSize > slowRadius_ )
			targetRotation = maxRotation_;
		else
			targetRotation = maxRotation_ * rotationSize / slowRadius_;

		targetRotation *= rotation / rotationSize;

		result.angular_ = targetRotation - character_.rotation_;
		result.angular_ /= timeToTarget_;

		float angularAccel = std::abs( result.angular_ );
		if ( angularAccel > maxAngularAccel_ )
		{
			result.angular_ /= angularAccel;
			result.angular_ *= maxAngularAccel_;
		}

		result.linear_ = 0;
		return result;
	}

};

class BlendedSteering
{
	using SteeringBehaviour = std::variant<Seek, Align>;

public:
	struct BehaviourAndWeight
	{
		SteeringBehaviour behaviour;
		float weight;
	};

	std::vector<BehaviourAndWeight *> behaviours;

	float maxAccel_;
	float maxRot_;

	SteeringOutput getSteering()
	{
		SteeringOutput result{};

		for ( auto b : behaviours )
		{
			SteeringOutput temp = std::visit( []( auto &b ) { return b.getSteering(); }, b->behaviour );
			result.linear_ += b->weight * temp.linear_;
			result.angular_ += b->weight * temp.angular_;
		}

		if ( result.linear_.length() > maxAccel_ )
		{
			result.linear_.normalise();
			result.linear_ *= maxAccel_;
		}

		float angularAccel = std::abs( result.angular_ );
		if ( angularAccel > maxRot_ )
		{
			result.angular_ /= angularAccel;
			result.angular_ *= maxRot_;
		}

		return result;
	}
};

int main( int argc, char *argv[] )
{
	int w{ WINDOW_WIDTH }, h{ WINDOW_HEIGHT };
	raylib::Window window( w, h, "Game AI: Assignment 1" );

	SetTargetFPS( 60 );

	raylib::AudioDevice ad;
	raylib::Sound deathSound = LoadSound( "../resources/sound.wav" );
	raylib::Music music = LoadMusicStream( "../resources/extremeaction.mp3" );
	music.looping = true;
	music.SetVolume( 0.2f );

	deathSound.SetVolume( 0.1f );

	Ship hunter{ w / 2.0f + 50, h / 2.0f, 0, RED };
	Ship prey{ w / 2.0f + 250, h / 2.0f + 300, 270 * DEG2RAD, BLUE };

	float target_radius{ 5 };
	float slow_radius{ 60 };
	const float max_accel{ 200 };
	const float max_ang_accel{ 10 };
	const float max_speed{ 220 };
	const float drag_factor{ 0.5 };
	const float max_rot{ 10.0f };

	Seek seek{ hunter.k_, prey.k_, max_accel };
	Wander wander{ prey.k_, max_accel, max_ang_accel };
	Align align{ hunter.k_, prey.k_, max_ang_accel, max_rot, target_radius, slow_radius };


	using BaW = BlendedSteering::BehaviourAndWeight;

	BaW seekW{ seek, 1.0f };
	BaW alignW{ align, 1.0f };

	std::vector<BaW *> behaviours;
	behaviours.push_back( &seekW );
	behaviours.push_back( &alignW );
	BlendedSteering blendSteer{ behaviours, max_accel, max_rot };


	PlayMusicStream( music );

	int deathCount = 0;

	while ( !window.ShouldClose() ) // Detect window close button or ESC key
	{
		UpdateMusicStream( music );

		BeginDrawing();

		ClearBackground( RAYWHITE );

		if ( IsMouseButtonPressed( MOUSE_LEFT_BUTTON ) )
		{
			const auto mpos = GetMousePosition();
		}

		DrawText( TextFormat( "Death Count: %03i", deathCount ), WINDOW_WIDTH / 2 - 80, 10, 20, RED );

		prey.draw( w, h );
		hunter.draw( w, h );

		EndDrawing();

		auto steer = blendSteer.getSteering();
		auto steer2 = wander.getSteering();
		hunter.k_.update( steer, max_speed, drag_factor, GetFrameTime() );
		prey.k_.update( steer2, max_speed - 60, drag_factor, GetFrameTime() );

		//Audio and respawning
		if ( ( hunter.k_.position_ - prey.k_.position_ ).length() <= 20 )
		{
			PlaySound( deathSound );

			std::uniform_real_distribution<float> widthDis( 0.0f, WINDOW_WIDTH );
			std::uniform_real_distribution<float> heightDis( 0.0f, WINDOW_HEIGHT );

			prey.k_.position_ = ( widthDis( seed ), 0.0f, heightDis( seed ) );
			//Not sure if required. Uncomment 
			//hunter.k_.position_ = (widthDis(seed), 0.0f, heightDis(seed));

			deathCount++;
		}
	}

	return 0;
}
