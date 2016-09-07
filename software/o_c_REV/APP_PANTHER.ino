// Copyright (c) 2016 Michael Hetrick
//
// Author: Michael Hetrick (michael [dot] s [dot] hetrick [at] gmail [dot] com)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// JAG-like cartesian CV plane.

#include "OC_apps.h"
#include "OC_menus.h"
#include "OC_strings.h"
#include "util/util_settings.h"

const uint32_t pantherZeroDistance = 4096;

enum PANTHER_SETTINGS
{
  PANTHER_SETTING_UNIBICVX,
  PANTHER_SETTING_UNIBICVY,
  PANTHER_SETTING_UNIBIOUT1,
  PANTHER_SETTING_UNIBIOUT2,
  PANTHER_SETTING_UNIBIOUT3,
  PANTHER_SETTING_UNIBIOUT4,
  PANTHER_NUM_SETTINGS
};

class PantherDac
{
  public:
    void init(int32_t _positionX, int32_t _positionY)
    {
      positionX = _positionX;
      positionY = _positionY;
    }

    //http://www.stm32duino.com/viewtopic.php?t=56
    uint32_t sqrt32(unsigned long n)
    {
      unsigned int c = 0x8000;
      unsigned int g = 0x8000;

      for(;;)
      {
          if(g*g > n)
          {
              g ^= c;
          }
          c >>= 1;
          if(c == 0)
          {
              return g;
          }
          g |= c;
      }
    }

    template <DAC_CHANNEL dacChannel>
    void update(const uint32_t _currentX, const uint32_t _currentY, const uint32_t _currentZ)
    {
      const int32_t xDiff = positionX - _currentX;
      const int32_t yDiff = positionY - _currentY;
      const uint32_t zDistance = _currentZ*_currentZ;
      const uint32_t summed = xDiff*xDiff + yDiff*yDiff + zDistance;

      outFactor = sqrt32(summed);
      CONSTRAIN(outFactor, 0, pantherZeroDistance);
      outFactor = pantherZeroDistance - outFactor;

      dacOut = uint32_t(outFactor * 16);
      OC::DAC::set<dacChannel>(dacOut);
    }
  private:
    uint32_t dacOut, outFactor;
    int32_t positionX, positionY;
};

class PantherApp : public settings::SettingsBase<PantherApp, PANTHER_NUM_SETTINGS>
{
  public:
    void Init()
    {
      InitDefaults();
      dac1.init(0, 4096);
      dac2.init(4096, 4096);
      dac3.init(0, 0);
      dac4.init(4096, 0);

      currentX = 0.0f;
      currentY = 0.0f;
    }

    void ISR()
    {
      cv_posx.push(OC::ADC::smoothed_raw_value(ADC_CHANNEL_1));
      cv_posy.push(OC::ADC::smoothed_raw_value(ADC_CHANNEL_2));
      cv_posz.push(OC::ADC::value<ADC_CHANNEL_3>());
      cv4.push(OC::ADC::value<ADC_CHANNEL_4>());

      currentX = 4096 - cv_posx.value();
      currentY = 4096 - cv_posy.value();
      incomingZ = cv_posz.value();
      currentZ = incomingZ > 0 ? incomingZ : 0;

      dac1.update<DAC_CHANNEL_A>(currentX, currentY, currentZ);
      dac2.update<DAC_CHANNEL_B>(currentX, currentY, currentZ);
      dac3.update<DAC_CHANNEL_C>(currentX, currentY, currentZ);
      dac4.update<DAC_CHANNEL_D>(currentX, currentY, currentZ);
    }

    void RenderScreensaver() const;

    // ISR update is at 16.666kHz, we don't need it that fast so smooth the values to ~1Khz
    static constexpr int32_t kSmoothing = 16;
    SmoothedValue<uint32_t, kSmoothing> cv_posx;
    SmoothedValue<uint32_t, kSmoothing> cv_posy;
    SmoothedValue<int32_t, kSmoothing> cv_posz;
    SmoothedValue<int32_t, kSmoothing> cv4;

  private:
    PantherDac dac1, dac2, dac3, dac4;
    uint32_t currentX, currentY, currentZ;
    int32_t incomingZ;
};


const char* const pantherUniBi[] =
{
  "Uni", "Bi"
};


SETTINGS_DECLARE(PantherApp, PANTHER_NUM_SETTINGS)
{
  { 1, 0, 1, "CV X", pantherUniBi, settings::STORAGE_TYPE_U8 },
  { 1, 0, 1, "CV Y", pantherUniBi, settings::STORAGE_TYPE_U8 },
  { 1, 0, 1, "Out 1", pantherUniBi, settings::STORAGE_TYPE_U8 },
  { 1, 0, 1, "Out 2", pantherUniBi, settings::STORAGE_TYPE_U8 },
  { 1, 0, 1, "Out 3", pantherUniBi, settings::STORAGE_TYPE_U8 },
  { 1, 0, 1, "Out 4", pantherUniBi, settings::STORAGE_TYPE_U8 },
};

PantherApp pantherApp;
struct{
  menu::ScreenCursor<menu::kScreenLines> cursor;
} pantherState;

// App stubs
void PANTHER_init()
{
  pantherState.cursor.Init(0, PANTHER_NUM_SETTINGS - 1);
  pantherApp.Init();
}

size_t PANTHER_storageSize()
{
  return PantherApp::storageSize();
}

size_t PANTHER_save(void *storage)
{
  return pantherApp.Save(storage);
}

size_t PANTHER_restore(const void *storage)
{
  return pantherApp.Restore(storage);
}

void FASTRUN PANTHER_isr()
{
  pantherApp.ISR();
}

void PANTHER_handleAppEvent(OC::AppEvent event)
{
  switch (event)
  {
    case OC::APP_EVENT_RESUME:
      break;
    case OC::APP_EVENT_SUSPEND:
    case OC::APP_EVENT_SCREENSAVER_ON:
    case OC::APP_EVENT_SCREENSAVER_OFF:
      break;
  }
}

void PANTHER_loop()
{

}

void PANTHER_menu()
{
  menu::DefaultTitleBar::Draw();

  menu::SettingsList<menu::kScreenLines, 0, menu::kDefaultValueX - 12> settings_list(pantherState.cursor);
  menu::SettingsListItem list_item;
  while (settings_list.available()) {
    const int current = settings_list.Next(list_item);
    list_item.DrawDefault(pantherApp.get_value(current), PantherApp::value_attr(current));
  }
}

void PantherApp::RenderScreensaver() const
{
  //128x64 screen
  //y 0 == top
  const uint16_t currentXVisual = currentX >> 7;
  const uint16_t currentYVisual = currentY >> 7;

  const uint16_t squareSize = 4;
  const uint16_t frameSize = 32;

  const uint16_t xPos = currentXVisual + 48;
  const uint16_t yPos = currentYVisual + 16;
  graphics.drawFrame(48, 16, frameSize, frameSize);
  graphics.drawRect(xPos - 2, 62 - yPos, squareSize, squareSize);
}

void PANTHER_screensaver()
{
  OC::scope_render();
  pantherApp.RenderScreensaver();
}

void PANTHER_topButton()
{

}

void PANTHER_lowerButton()
{

}

void PANTHER_rightButton()
{
  pantherState.cursor.toggle_editing();
}

void PANTHER_leftButton()
{

}

void PANTHER_handleButtonEvent(const UI::Event &event)
{
  if (UI::EVENT_BUTTON_PRESS == event.type)
  {
    switch (event.control)
    {
      case OC::CONTROL_BUTTON_UP:
        PANTHER_topButton();
        break;
      case OC::CONTROL_BUTTON_DOWN:
        PANTHER_lowerButton();
        break;
      case OC::CONTROL_BUTTON_L:
        PANTHER_leftButton();
        break;
      case OC::CONTROL_BUTTON_R:
        PANTHER_rightButton();
        break;
    }
  }
}

void PANTHER_handleEncoderEvent(const UI::Event &event)
{
  if (OC::CONTROL_ENCODER_L == event.control)
  {

  }
  else if (OC::CONTROL_ENCODER_R == event.control)
  {
    if (pantherState.cursor.editing())
    {
      pantherApp.change_value(pantherState.cursor.cursor_pos(), event.value);
    }
    else
    {
      pantherState.cursor.Scroll(event.value);
    }
  }
}
