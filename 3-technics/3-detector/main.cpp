////////////////////////////////////////////////////////////////
//    using c++20, therefor ignore some warnings              //
////////////////////////////////////////////////////////////////
//#define SPDLOG_FMT_EXTERNAL

#include <bits/stdint-uintn.h>
#include <cstddef>
#include <fmt/core.h>
#include <spdlog/spdlog.h>
#include <docopt/docopt.h>
#include <nlohmann/json.hpp>
#include <fstream>
#include <string>
#include <map>
#include <iostream>
#include <variant>
#include <chrono>
extern "C" {
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>
}
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "input.hpp"
#include "util/imguihelper.hpp"
#include "entt/entt.hpp"
#include "loader.hpp"
#include "components.hpp"


static constexpr auto USAGE =
        // raw string literal
        R"(War And Dragons

  Usage:
       war                 [options]

  Options:
       -h --help           Show this screen.
       --width=WIDTH       Screen width in pixels [default: 1024].
       --height=HEIGHT     Screen height in pixels [default: 768].
       --scale=SCALE       Scaling factor [default: 2].
       --replay=EVENTFILE  JSON file of events to play

 )";


int main (int argc, const char** argv) {

    ////////////////////////////////////////////////////////////////
    // Resource Management
    //
    ////////////////////////////////////////////////////////////////
    entt::resource_cache<sf::Texture> t_cache;
    t_cache.load<my_loader<sf::Texture>>(Textures::Treasure_Chest, "textures/tc");

    t_cache.each ([](const id_type id, const entt::resource_handle<sf::Texture> &t_handle){
        const auto s_vec = t_handle->getSize();
        fmt::print("Resource id: {}, width: {}, height: {}\n", id, s_vec.x, s_vec.y);
    });


    ////////////////////////////////////////////////////////////////
    // Entity Component System
    //
    ////////////////////////////////////////////////////////////////
    entt::registry registry;
    entt::entity treasure_chest = registry.create();
    if constexpr (std::is_integral_v<entt::entity>){
        fmt::print("entity integral type, treasure chest id: {}\n", treasure_chest);
    } else if constexpr (std::is_enum_v<entt::entity>) {
        fmt::print("entity_type: enum\n");
    }
    registry.emplace<Transform>(treasure_chest, sf::Vector2f{}, 0);
    registry.emplace<StaticRender>(treasure_chest);

    auto &chest_sprite = registry.get<StaticRender>(treasure_chest).sprite;
    chest_sprite.setTexture(t_cache.handle(Textures::Treasure_Chest).get());
    chest_sprite.setScale({0.2, 0.2});
    chest_sprite.setPosition({50, 200});


    ////////////////////////////////////////////////////////////////
    // Config
    //
    ////////////////////////////////////////////////////////////////
    std::map<std::string, docopt::value> args = docopt::docopt(USAGE, {std::next (argv), std::next (argv, argc)}, true, "Game 0.0");

    // damit überprüfung auf ungültige eingaben auch auf negativ
    for (auto const &arg : args) {
        if (arg.second.isString()) {
            spdlog::info("Parameter set: {}='{}'", arg.first, arg.second.asString());
        }
    }

    const auto width = args["--width"].asLong();
    const auto height = args["--height"].asLong();
    const auto scale = args["--scale"].asLong();

    std::vector<Game::GameState::Event> initalEvents;
    if (args["--replay"]) {
        const auto eventFile = args["--replay"].asString();
        std::ifstream ifs(eventFile);
        const auto j = nlohmann::json::parse(ifs);
        initalEvents = j.get<std::vector<Game::GameState::Event>>();
    }

    if (width < 0 || height < 0 || scale < 1 || scale > 5) {

        spdlog::error("Command line option sre out of reasonable range.");
        for (auto const &arg : args) std::cout << arg.first << arg.second << '\n';
        abort();

    }

    spdlog::set_level(spdlog::level::debug);
    spdlog::info ("Starting ImGUI + SFML");


    ////////////////////////////////////////////////////////////////
    // LUA
    //
    ////////////////////////////////////////////////////////////////
    lua_State *lstate = luaL_newstate();
    luaL_openlibs(lstate);


    ////////////////////////////////////////////////////////////////
    // Create Context
    //
    ////////////////////////////////////////////////////////////////
    sf::RenderWindow window(sf::VideoMode(static_cast<unsigned int>(width), static_cast<unsigned int>(height)), "War And Dragons");
    ImGui::SFML::Init(window);

    const auto scale_factor = static_cast<float>(scale);
    ImGui::GetStyle().ScaleAllSizes (scale_factor);
    ImGui::GetIO().FontGlobalScale = scale_factor;

    Game::GameState gs;
    gs.setEvents(initalEvents);

    constexpr std::array topics = {
            "The Plan",
            "Getting Started",
            "Handling Command Line Parameters",
            "Reading SFML Input States",
            "C++ So Far",
            "Managing Game State",
            "Making Our Game Testable",
            "Add Logging To Game Engine",
            "Draw A Game Map",
            "Dialog Trees",
            "Porting From SFML To SDL"
    };
    std::array<bool, topics.size()> states{ false };
    bool joystickEvent{ false };
    std::uint64_t eventsProcessed{0};
    std::vector<Game::GameState::Event> events{ Game::GameState::TimeElapsed{} };


    ////////////////////////////////////////////////////////////////
    // Timings
    //
    ////////////////////////////////////////////////////////////////
    double t = 0.0;
    double dt = 0.01;

    auto currentTime = std::chrono::steady_clock::now();
    double accumulator = 0.0;

    // State previous;
    // State current;


    ////////////////////////////////////////////////////////////////
    // Game Loop
    //
    ////////////////////////////////////////////////////////////////
    while (window.isOpen()) {

        const auto newTime = std::chrono::steady_clock::now();

        double frameTime = duration_cast<std::chrono::milliseconds>(newTime - currentTime).count();

        if (frameTime > 0.25)
        {
            frameTime = 0.25;
        }
        currentTime = newTime;
        accumulator += frameTime;


        ////////////////////////////////////////////////////////////////
        // Event HAndling
        //
        ////////////////////////////////////////////////////////////////
        const auto event = gs.nextEvent(window);

        // compressing TimeElapsed Events
        // and recording ongoing events
        std::visit(Game::overloaded{
                           [](Game::GameState::TimeElapsed& prev, const Game::GameState::TimeElapsed& next) {
                               prev.elapsed += next.elapsed;
                           },
                           [&] (const auto& /*prev*/, const std::monostate& /*next*/) {},
                           [&] (const auto& /*prev*/, const auto& next) {
                               events.push_back(next);
                           }
                   },
                   events.back(),
                   event);

        ++eventsProcessed;

        if(const auto sfmlEvent = Game::GameState::toSFMLEvent(event); sfmlEvent) {
            // std::optional
            ImGui::SFML::ProcessEvent(*sfmlEvent);
        }

        bool timeElapsed{false};
        std::visit(Game::overloaded{
                [&] (const Game::JoyStickEvent auto& jsEvent) {
                    gs.update(jsEvent);
                    joystickEvent = true;
                },
                [&] (const Game::GameState::CloseWindow& /*unused*/) { window.close(); },
                [&] (const Game::GameState::TimeElapsed& te) {
                    ImGui::SFML::Update(window, te.toSFMLTime());
                    timeElapsed = true;
                },
                [&] (const auto& /*do nothing*/) {},
                [&] (const std::monostate& /*unused*/) { /*spdlog::debug("monostate");*/ }
        }, event);

        if (!timeElapsed) {
            // todo: something more with a linear flow here
            // right now this is just saying "no reason to update the render yet"
            continue;
        }


        ////////////////////////////////////////////////////////////////
        //                           Update                           //
        ////////////////////////////////////////////////////////////////
        while (accumulator >= dt)
        {

            // previousState = currentState;
            // integrate (currentState, t, dt);

            t += dt;
            accumulator -= dt;

        }


        ////////////////////////////////////////////////////////////////
        //                           Render                           //
        ////////////////////////////////////////////////////////////////
        const double alpha = accumulator / dt;

        // interpolate
        // State state = currentState * alpha +
        //   previousState * (1.0 - alpha);

        // render (state);


        ImGui::Begin ("The Plan");
        for (size_t index = 0; auto const &steps : topics) {
            ImGui::Checkbox (fmt::format("{} : {}", index, steps).c_str(), &states.at(index));
            ++index;
        }
        ImGui::End(); // end window


        ImGui::Begin("Joystick");
        if (!gs.joySticks.empty()) {
            ImGuiHelper::Text("Joystick Event: {}", joystickEvent);
            joystickEvent = false;
            for (size_t button = 0; button < gs.joySticks[0].buttonCount; button++) {
                ImGuiHelper::Text("{}: {}", button, gs.joySticks[0].buttonState[button]);
            }
            for (size_t axis = 0; axis < sf::Joystick::AxisCount; axis++) {
                ImGuiHelper::Text("{}: {}", Game::GameState::toString (static_cast<sf::Joystick::Axis>(axis)), gs.joySticks[0].axisPosition[axis]);
            }
        }
        ImGui::End();


        window.clear(); // fill background with color
        window.draw(registry.get<StaticRender>(treasure_chest).sprite);
        ImGui::SFML::Render(window);
        window.display();

    }


    ////////////////////////////////////////////////////////////////
    //                           Close                            //
    ////////////////////////////////////////////////////////////////
    ImGui::SFML::Shutdown();

    spdlog::info("Total events processed: {}, total recorded: {}", eventsProcessed, events.size());

    // for (const auto& event : events) {
    //   std::visit (Game::overloaded{
    // 		[] (const auto& event_obj) {
    // 		  spdlog::info("Event: {}", event_obj.name);
    // 		},
    // 		[] (const std::monostate& /*nothing*/) {}
    //     }, event);
    // }

    nlohmann::json serialized(events);

    // auto first_time_elapsed = serialized[0].get<Game::GameState::TimeElapsed>();
    // spdlog::info("First time elapsed: {}", first_time_elapsed.elapsed.count());

    std::ofstream ofs{ "events.json" };
    ofs << serialized.dump(2);

    return EXIT_SUCCESS;
}
