#include "menu.h"

static MenuItem *Menu_GetHead(MenuItem *item)
{
    if (item == NULL) {
        return NULL;
    }

    while (item->prev != NULL) {
        item = item->prev;
    }

    return item;
}

static uint8_t Menu_CountItems(MenuItem *head)
{
    uint8_t count = 0U;

    while (head != NULL) {
        count++;
        head = head->next;
    }

    return count;
}

static uint8_t Menu_GetIndex(MenuItem *head, MenuItem *item)
{
    uint8_t index = 0U;

    while ((head != NULL) && (head != item)) {
        head = head->next;
        index++;
    }

    return index;
}

static MenuItem *Menu_GetItemAt(MenuItem *head, uint8_t index)
{
    while ((head != NULL) && (index > 0U)) {
        head = head->next;
        index--;
    }

    return head;
}

static uint8_t Menu_ClampWindowStart(uint8_t count, uint8_t max_show_items, uint8_t start_idx)
{
    if ((count == 0U) || (count <= max_show_items)) {
        return 0U;
    }

    if (start_idx > (uint8_t)(count - max_show_items)) {
        start_idx = (uint8_t)(count - max_show_items);
    }

    return start_idx;
}

static void Menu_ApplySelection(MenuManager *nav, MenuItem *head, uint8_t absolute_index,
    uint8_t window_start_hint)
{
    uint8_t count;
    uint8_t window_start;

    if ((nav == NULL) || (head == NULL) || (nav->max_show_items == 0U)) {
        return;
    }

    count = Menu_CountItems(head);
    if (count == 0U) {
        return;
    }

    if (absolute_index >= count) {
        absolute_index = (uint8_t)(count - 1U);
    }

    window_start = Menu_ClampWindowStart(count, nav->max_show_items, window_start_hint);

    if (absolute_index < window_start) {
        window_start = absolute_index;
    } else if (absolute_index >= (uint8_t)(window_start + nav->max_show_items)) {
        window_start = (uint8_t)(absolute_index - nav->max_show_items + 1U);
    }

    window_start = Menu_ClampWindowStart(count, nav->max_show_items, window_start);

    nav->current_menu = head;
    nav->cursor_item = Menu_GetItemAt(head, absolute_index);
    nav->show_start_idx = window_start;
    nav->cursor_index = (uint8_t)(absolute_index - window_start);
}

void Menu_Init(MenuManager *nav, MenuItem *root_menu, uint8_t max_items_on_screen)
{
    if ((nav == NULL) || (root_menu == NULL)) {
        return;
    }

    nav->max_show_items = max_items_on_screen;
    Menu_ApplySelection(nav, Menu_GetHead(root_menu), 0U, 0U);
}

void Menu_Up(MenuManager *nav)
{
    MenuItem *head;
    uint8_t count;
    uint8_t index;

    if ((nav == NULL) || (nav->cursor_item == NULL)) {
        return;
    }

    head = Menu_GetHead(nav->current_menu);
    count = Menu_CountItems(head);
    if (count == 0U) {
        return;
    }

    index = Menu_GetIndex(head, nav->cursor_item);
    if (index == 0U) {
        index = (uint8_t)(count - 1U);
    } else {
        index--;
    }

    Menu_ApplySelection(nav, head, index, nav->show_start_idx);
}

void Menu_Down(MenuManager *nav)
{
    MenuItem *head;
    uint8_t count;
    uint8_t index;

    if ((nav == NULL) || (nav->cursor_item == NULL)) {
        return;
    }

    head = Menu_GetHead(nav->current_menu);
    count = Menu_CountItems(head);
    if (count == 0U) {
        return;
    }

    index = (uint8_t)(Menu_GetIndex(head, nav->cursor_item) + 1U);
    if (index >= count) {
        index = 0U;
        Menu_ApplySelection(nav, head, index, 0U);
    } else {
        Menu_ApplySelection(nav, head, index, nav->show_start_idx);
    }
}

void Menu_Enter(MenuManager *nav)
{
    MenuItem *child_head;
    uint8_t child_index;

    if ((nav == NULL) || (nav->cursor_item == NULL)) {
        return;
    }

    if (nav->cursor_item->child != NULL) {
        child_head = Menu_GetHead(nav->cursor_item->child);
        child_index = Menu_GetIndex(child_head, nav->cursor_item->child);
        Menu_ApplySelection(nav, child_head, child_index, 0U);
    } else if (nav->cursor_item->callback != NULL) {
        nav->cursor_item->callback();
    }
}

void Menu_Back(MenuManager *nav)
{
    MenuItem *parent_item;
    MenuItem *head;
    uint8_t parent_index;

    if ((nav == NULL) || (nav->current_menu == NULL)) {
        return;
    }

    if (nav->current_menu->parent != NULL) {
        parent_item = nav->current_menu->parent;
        head = Menu_GetHead(parent_item);
        parent_index = Menu_GetIndex(head, parent_item);
        Menu_ApplySelection(nav, head, parent_index, parent_index);
    }
}
